from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from ..models.query import Query
from ..models.query_result import QueryResult
from ..config.database import qdrant_client
from .embedding_service import EmbeddingService
from .document_service import DocumentService
import uuid
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.embedding_service = EmbeddingService()

    def process_query(self, db: Session, query_text: str, selected_text: Optional[str] = None, session_id: Optional[str] = None) -> QueryResult:
        """
        Process a user query using the RAG system
        """
        # Create query record
        query_id = str(uuid.uuid4())
        query = Query(
            id=query_id,
            query_text=query_text,
            selected_text=selected_text,
            session_id=session_id or str(uuid.uuid4()),
            context_json="{}"  # Empty context for now
        )
        db.add(query)
        db.commit()

        # Generate embeddings for the query
        query_embedding = self.embedding_service.generate_embeddings(query_text)

        # Search for similar content in the vector store
        search_results = self.embedding_service.search_similar(query_embedding, limit=5)

        # Generate response based on retrieved content
        response_text = self._generate_response(query_text, selected_text, search_results)

        # Create query result record
        query_result = QueryResult(
            id=str(uuid.uuid4()),
            query_id=query_id,
            response_text=response_text,
            source_documents_json=str([result["document_id"] for result in search_results]),
            confidence_score=0.85,  # Placeholder confidence score
            retrieved_chunks_json=str([{"id": str(i), "text": result["chunk_text"][:100] + "..."} for i, result in enumerate(search_results)])
        )
        db.add(query_result)
        db.commit()

        return query_result

    def _generate_response(self, query_text: str, selected_text: Optional[str], search_results: List[Dict[str, Any]]) -> str:
        """
        Generate a response based on the query and retrieved results using OpenAI
        """
        from ..agents.skills import AgentSkills
        agent = AgentSkills()
        
        if not search_results:
            return agent.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a helpful AI assistant for a robotics textbook. The user asked a question but I couldn't find relevant content in the book. Answer politely based on your general knowledge, but mention that it's outside the book's scope."},
                    {"role": "user", "content": query_text}
                ]
            ).choices[0].message.content

        # Combine the retrieved results to form a context
        context = "\n\n".join([result["chunk_text"] for result in search_results[:3]])  # Use top 3 results

        system_prompt = f"""
        You are an expert AI tutor for a Physical AI & Robotics textbook.
        Use the following retrieved context from the book to answer the user's question.
        
        Context:
        {context}
        
        If selected text is provided, focus the answer on that specific section.
        """
        
        user_prompt = f"Question: {query_text}"
        if selected_text:
            user_prompt += f"\n\nSelected Text: {selected_text}"

        response = agent.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ]
        )

        return response.choices[0].message.content

    def index_document(self, db: Session, document: Any) -> bool:
        """
        Index a document by chunking its content and storing embeddings
        """
        try:
            # Chunk the document content
            chunks = self.embedding_service.chunk_document_content(document.content)

            # Process each chunk
            for chunk in chunks:
                # Generate embeddings for the chunk
                chunk_embedding = self.embedding_service.generate_embeddings(chunk["text"])

                # Store the embedding in Qdrant
                self.embedding_service.create_embedding(
                    document_id=document.id,
                    chunk_text=chunk["text"],
                    chunk_index=chunk["index"],
                    vector=chunk_embedding,
                    metadata={
                        "chapter_number": document.chapter_number,
                        "module": document.module,
                        "title": document.title
                    }
                )

            logger.info(f"Successfully indexed document {document.id} with {len(chunks)} chunks")
            return True
        except Exception as e:
            logger.error(f"Error indexing document {document.id}: {str(e)}")
            return False