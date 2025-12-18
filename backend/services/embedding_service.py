import cohere
import os
from dotenv import load_dotenv
from typing import List

# Load environment variables
load_dotenv()

class EmbeddingService:
    def __init__(self):
        # Initialize Cohere client with the API key from environment
        self.cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
        self.embedding_model = "embed-english-v3.0"  # As specified in the requirements (1024 dims)

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using Cohere embed-english-v3.0 model
        Returns a 1024-dimensional vector as required
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model=self.embedding_model,
                input_type="search_document"  # Appropriate input type for document search
            )

            # Extract the embedding (first item in the response)
            embedding = response.embeddings[0]

            # Verify it's 1024 dimensions as required
            if len(embedding) != 1024:
                raise ValueError(f"Expected 1024-dimensional embedding, got {len(embedding)} dimensions")

            return embedding
        except Exception as e:
            raise Exception(f"Error generating embedding: {str(e)}")

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        """
        try:
            response = self.cohere_client.embed(
                texts=texts,
                model=self.embedding_model,
                input_type="search_document"
            )

            embeddings = response.embeddings

            # Verify all embeddings are 1024 dimensions
            for i, embedding in enumerate(embeddings):
                if len(embedding) != 1024:
                    raise ValueError(f"Text {i}: Expected 1024-dimensional embedding, got {len(embedding)} dimensions")

            return embeddings
        except Exception as e:
            raise Exception(f"Error generating embeddings: {str(e)}")