import qdrant_client
from qdrant_client.http import models
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class VectorSetup:
    def __init__(self):
        # Initialize Qdrant client with the host and API key from environment
        self.host = os.getenv("QDRANT_HOST")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "biblio_chat")

        self.client = qdrant_client.QdrantClient(
            url=self.host,
            api_key=self.api_key,
        )

    def setup_collection(self):
        """
        Check if collection exists, and if not, create it with required specifications:
        - vector_size: 1024 (Strict requirement for Cohere embed-english-v3.0)
        - distance: Cosine
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name in collection_names:
                print(f"Collection '{self.collection_name}' already exists.")
                return True

            # Create collection with 1024-dimensional vectors and cosine distance
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1024,  # Required dimension for Cohere embed-english-v3.0
                    distance=models.Distance.COSINE
                )
            )

            print(f"Collection '{self.collection_name}' created successfully with 1024-dimensional vectors and cosine distance.")
            return True

        except Exception as e:
            print(f"Error setting up collection: {str(e)}")
            return False

if __name__ == "__main__":
    vector_setup = VectorSetup()
    vector_setup.setup_collection()