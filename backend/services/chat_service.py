import cohere
from typing import List
from models.request import ChatRequest
from models.response import ChatResponse
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class ChatService:
    def __init__(self):
        # Initialize Cohere client with the API key from environment
        self.cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
        self.chat_model = "command-r"  # As specified in the requirements

    async def process_request(self, chat_request: ChatRequest) -> ChatResponse:
        """
        Process the chat request using dual-mode logic:
        - Selection Mode: If selection_context is provided, bypasses vector search and sends text directly to AI
        - Global Mode: If selection_context is None, would perform vector search (to be implemented with Qdrant)
        """
        if chat_request.selection_context:
            # Selection mode: Use the provided context directly
            return await self._process_selection_mode(chat_request)
        else:
            # Global mode: This would involve Qdrant search (to be implemented)
            return await self._process_global_mode(chat_request)

    async def _process_selection_mode(self, chat_request: ChatRequest) -> ChatResponse:
        """
        Process in selection mode - send the selected text context directly to Cohere
        """
        try:
            # Construct the prompt with the selection context
            prompt = f"You are a helpful assistant. Analyze this specific text: {chat_request.selection_context}. Question: {chat_request.query}."

            # Call Cohere chat endpoint
            response = self.cohere_client.chat(
                message=prompt,
                model=self.chat_model
            )

            # Extract the response text
            ai_response = response.text

            # Return the response with empty sources (since we're not using RAG in selection mode)
            return ChatResponse(response=ai_response, sources=[])

        except Exception as e:
            raise Exception(f"Error in selection mode processing: {str(e)}")

    async def _process_global_mode(self, chat_request: ChatRequest) -> ChatResponse:
        """
        Process in global mode - involves vector search in Qdrant and then sending to Cohere
        """
        try:
            import sys
            import os
            sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

            from services.embedding_service import EmbeddingService
            from qdrant_client import QdrantClient
            from dotenv import load_dotenv

            load_dotenv()

            # Initialize services
            embedding_service = EmbeddingService()
            qdrant_client = QdrantClient(
                url=os.getenv("QDRANT_HOST"),
                api_key=os.getenv("QDRANT_API_KEY"),
            )
            collection_name = os.getenv("QDRANT_COLLECTION_NAME", "biblio_chat")

            # 1. Embed the query using Cohere embed-english-v3.0
            query_embedding = embedding_service.embed_text(chat_request.query)

            # 2. Search Qdrant for the top 3 matches using the query vector
            search_results = qdrant_client.search(
                collection_name=collection_name,
                query_vector=query_embedding,
                limit=3,
                with_payload=True
            )

            # 3. Extract context from the search results
            context_parts = []
            sources = []
            for result in search_results:
                if result.payload and 'text_content' in result.payload:
                    context_parts.append(result.payload['text_content'])
                    # Add source identifier to the sources list
                    if 'source_file' in result.payload:
                        sources.append(result.payload['source_file'])
                    else:
                        sources.append(f"Chunk {result.payload.get('chunk_index', 'unknown')}")

            if not context_parts:
                # If no context found, fall back to a simple response
                ai_response = f"I couldn't find relevant information in the book to answer: {chat_request.query}"
                return ChatResponse(response=ai_response, sources=[])

            # Construct context string
            context = "\n\n".join(context_parts)

            # 4. Construct system prompt with retrieved context
            prompt = f"Use the following context to answer the question. Context: {context}. Question: {chat_request.query}."

            # 5. Call Cohere 'command-r' chat endpoint
            response = self.cohere_client.chat(
                message=prompt,
                model=self.chat_model
            )

            # Extract the response text
            ai_response = response.text

            # Return the response with sources
            return ChatResponse(response=ai_response, sources=sources)

        except Exception as e:
            raise Exception(f"Error in global mode processing: {str(e)}")