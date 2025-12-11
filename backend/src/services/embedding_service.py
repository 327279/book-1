from typing import List, Optional
from qdrant_client import models
from ..config.database import qdrant_client
from ..models.document import Document
import uuid
import openai
from ..config.settings import settings

class EmbeddingService:
    def __init__(self):
        self.collection_name = "book_content"
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """
        Create the Qdrant collection if it doesn't exist
        """
        try:
            qdrant_client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # Assuming OpenAI embeddings
            )

    def create_embedding(self, document_id: str, chunk_text: str, chunk_index: int, vector: List[float], metadata: dict = None) -> str:
        """
        Create an embedding in Qdrant
        """
        point_id = str(uuid.uuid4())

        qdrant_client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={
                        "document_id": document_id,
                        "chunk_text": chunk_text,
                        "chunk_index": chunk_index,
                        "metadata": metadata or {}
                    }
                )
            ]
        )

        return point_id

    def search_similar(self, query_vector: List[float], limit: int = 5) -> List[dict]:
        """
        Search for similar content based on the query vector
        """
        results = qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit
        )

        return [
            {
                "document_id": result.payload["document_id"],
                "chunk_text": result.payload["chunk_text"],
                "chunk_index": result.payload["chunk_index"],
                "metadata": result.payload["metadata"],
                "score": result.score
            }
            for result in results
        ]

    def delete_document_embeddings(self, document_id: str) -> bool:
        """
        Delete all embeddings associated with a specific document
        """
        # Find all points with the given document_id
        search_results = qdrant_client.scroll(
            collection_name=self.collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.document_id",
                        match=models.MatchValue(value=document_id)
                    )
                ]
            ),
            limit=10000  # Assuming max 10k chunks per document
        )

        point_ids = [point.id for point in search_results[0]]

        if point_ids:
            qdrant_client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=point_ids)
            )
            return True

        return False

    def chunk_document_content(self, content: str, chunk_size: int = 1000, overlap: int = 100) -> List[dict]:
        """
        Chunk document content into smaller pieces for embedding
        """
        chunks = []
        start = 0

        while start < len(content):
            end = start + chunk_size

            # If we're near the end, make sure to include the rest
            if end > len(content):
                end = len(content)
            else:
                # Try to break at sentence boundary
                while end < len(content) and content[end] not in '.!?。！？\n' and end - start < chunk_size + 50:
                    end += 1
                if end < len(content):
                    end += 1  # Include the sentence ending punctuation

            chunk_text = content[start:end].strip()

            if len(chunk_text) > 0:  # Only add non-empty chunks
                chunks.append({
                    "text": chunk_text,
                    "index": len(chunks)
                })

            # Move start position, with overlap if possible
            start = end - overlap if end - overlap > start else end

            # If we're not making progress, move forward by chunk_size to avoid infinite loop
            if start == end - overlap and overlap > 0:
                start = end

        return chunks

    def generate_embeddings(self, text: str) -> List[float]:
        """
        Generate embeddings for text using OpenAI's embedding API
        """
        # Initialize OpenAI client with API key from settings
        client = openai.OpenAI(api_key=settings.openai_api_key)

        try:
            # Call OpenAI embeddings API
            response = client.embeddings.create(
                input=text,
                model="text-embedding-ada-002"
            )

            # Extract the embedding from the response
            embedding = response.data[0].embedding
            return embedding
        except Exception as e:
            print(f"Error generating embeddings: {str(e)}")
            # Return a default embedding in case of error
            return [0.0] * 1536