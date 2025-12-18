import os
from dotenv import load_dotenv
from services.embedding_service import EmbeddingService
from vector_setup import VectorSetup
import qdrant_client
from qdrant_client.http import models
import uuid

# Load environment variables
load_dotenv()

class IngestionService:
    def __init__(self):
        # Initialize services
        self.embedding_service = EmbeddingService()
        self.vector_setup = VectorSetup()

        # Initialize Qdrant client
        self.host = os.getenv("QDRANT_HOST")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "biblio_chat")

        self.qdrant_client = qdrant_client.QdrantClient(
            url=self.host,
            api_key=self.api_key,
        )

    def read_book_content(self, file_path: str = "book_source.txt") -> str:
        """
        Read book content from a text file
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()
            return content
        except FileNotFoundError:
            # If the file doesn't exist, create a sample one for testing
            sample_content = """
            This is a sample book content for testing the BiblioChat backend. The AI assistant will be able to answer questions about this text content using the RAG system. You can replace this with actual book content for real usage.

            Chapter 1: Introduction to AI
            Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.

            Chapter 2: Machine Learning Basics
            Machine Learning is a subset of AI that focuses on algorithms that can learn from data. There are several types of machine learning including supervised learning, unsupervised learning, and reinforcement learning.

            Chapter 3: Natural Language Processing
            Natural Language Processing (NLP) is a field of AI focused on the interaction between computers and humans through natural language. The ultimate objective of NLP is to read, decipher, understand, and make sense of human languages in a manner that is valuable.
            """
            with open(file_path, 'w', encoding='utf-8') as file:
                file.write(sample_content)
            return sample_content

    def chunk_text(self, text: str, chunk_size: int = 1000, overlap: int = 100) -> list:
        """
        Split text into chunks of approximately chunk_size characters with overlap
        """
        chunks = []
        start = 0

        while start < len(text):
            # Determine the end position
            end = start + chunk_size

            # If this is the last chunk, include all remaining text
            if end >= len(text):
                chunk = text[start:]
            else:
                # Try to break at sentence boundary within the overlap region
                chunk = text[start:end]

                # Find a good breaking point (sentence end, paragraph end, or word boundary)
                if end < len(text):
                    # Look for a good break point in the overlap region
                    break_found = False
                    for i in range(min(overlap, len(chunk)), 0, -1):
                        if text[start + i] in '.!?':
                            chunk = text[start:start + i + 1]
                            end = start + i + 1
                            break_found = True
                            break

                    if not break_found:
                        # If no sentence boundary found, try word boundary
                        for i in range(min(overlap, len(chunk)), 0, -1):
                            if text[start + i] == ' ':
                                chunk = text[start:start + i]
                                end = start + i
                                break_found = True
                                break

            chunks.append(chunk.strip())

            # Move start to the next position
            if break_found:
                start = end
            else:
                start += chunk_size

            # If we've processed all text, break
            if start >= len(text):
                break

        # Filter out empty chunks
        chunks = [chunk for chunk in chunks if chunk and len(chunk.strip()) > 0]

        return chunks

    def ingest_book(self, file_path: str = "book_source.txt"):
        """
        Main method to ingest a book: read content, chunk it, embed it, and upsert to Qdrant
        """
        try:
            # Read book content
            print("Reading book content...")
            content = self.read_book_content(file_path)

            # Chunk the content
            print("Chunking text...")
            chunks = self.chunk_text(content)
            print(f"Created {len(chunks)} chunks")

            # Embed each chunk
            print("Generating embeddings...")
            embeddings = []
            for i, chunk in enumerate(chunks):
                embedding = self.embedding_service.embed_text(chunk)
                embeddings.append(embedding)
                if (i + 1) % 10 == 0:  # Progress indicator
                    print(f"Embedded {i + 1}/{len(chunks)} chunks")

            # Prepare points for Qdrant
            print("Preparing points for Qdrant...")
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point = models.PointStruct(
                    id=str(uuid.uuid4()),  # Generate unique ID
                    vector=embedding,
                    payload={
                        "text_content": chunk,
                        "chunk_index": i,
                        "source_file": file_path
                    }
                )
                points.append(point)

            # Upsert to Qdrant
            print("Upserting vectors to Qdrant...")
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            print(f"Successfully ingested {len(chunks)} chunks to Qdrant collection '{self.collection_name}'")
            return True

        except Exception as e:
            print(f"Error during ingestion: {str(e)}")
            return False

if __name__ == "__main__":
    # Ensure the collection exists
    vector_setup = VectorSetup()
    vector_setup.setup_collection()

    # Perform ingestion
    ingestion_service = IngestionService()
    ingestion_service.ingest_book()