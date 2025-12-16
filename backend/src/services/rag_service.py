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

    def _detect_topic(self, query_text: str) -> Optional[str]:
        """
        Detect the topic/chapter focus of a query for better filtering
        """
        query_lower = query_text.lower()
        
        # Topic keywords mapping
        topic_keywords = {
            "ros2": ["ros", "ros2", "node", "topic", "service", "rclpy", "urdf", "publisher", "subscriber"],
            "simulation": ["gazebo", "unity", "simulation", "physics", "sensor", "lidar", "imu", "depth camera"],
            "isaac": ["isaac", "nvidia", "nav2", "vslam", "path planning", "navigation"],
            "vla": ["vla", "voice", "whisper", "natural language", "cognitive", "speech"],
            "capstone": ["capstone", "autonomous", "integration", "manipulation", "object identification"],
            "conversational": ["conversational", "dialogue", "interaction", "chat", "conversation"],
            "humanoid": ["humanoid", "robot", "bipedal", "balance", "locomotion"]
        }
        
        # Count matches for each topic
        topic_scores = {}
        for topic, keywords in topic_keywords.items():
            score = sum(1 for keyword in keywords if keyword in query_lower)
            if score > 0:
                topic_scores[topic] = score
        
        # Return topic with highest score
        if topic_scores:
            return max(topic_scores, key=topic_scores.get)
        return None

    def process_query(self, db: Session, query_text: str, selected_text: Optional[str] = None, session_id: Optional[str] = None) -> QueryResult:
        """
        Process a user query using the RAG system with topic-specific enhancement
        """
        # Create query record
        query_id = str(uuid.uuid4())
        
        # Detect query topic for better context
        detected_topic = self._detect_topic(query_text)
        
        query = Query(
            id=query_id,
            query_text=query_text,
            selected_text=selected_text,
            session_id=session_id or str(uuid.uuid4()),
            context_json=str({"detected_topic": detected_topic}) if detected_topic else "{}"
        )
        db.add(query)
        db.commit()

        # Generate embeddings for the query
        query_embedding = self.embedding_service.generate_embeddings(query_text)

        # Search for similar content in the vector store
        # Increase limit to filter by topic later
        search_results = self.embedding_service.search_similar(query_embedding, limit=10)
        
        # Filter results by detected topic if available
        if detected_topic and search_results:
            filtered_results = [
                result for result in search_results 
                if detected_topic in str(result.get("metadata", {})).lower()
            ]
            # Use filtered results if we have enough, otherwise fall back to all results
            if len(filtered_results) >= 3:
                search_results = filtered_results[:5]
            else:
                search_results = search_results[:5]
        else:
            search_results = search_results[:5]

        # Generate response based on retrieved content
        response_text = self._generate_response(query_text, selected_text, search_results, detected_topic)

        # Create query result record
        query_result = QueryResult(
            id=str(uuid.uuid4()),
            query_id=query_id,
            response_text=response_text,
            source_documents_json=str([result["document_id"] for result in search_results]),
            confidence_score=0.90 if detected_topic else 0.85,  # Higher confidence for topic-specific queries
            retrieved_chunks_json=str([{"id": str(i), "text": result["chunk_text"][:100] + "..."} for i, result in enumerate(search_results)])
        )
        db.add(query_result)
        db.commit()

        return query_result

    def _generate_response(self, query_text: str, selected_text: Optional[str], search_results: List[Dict[str, Any]], detected_topic: Optional[str] = None) -> str:
        """
        Generate a response based on the query and retrieved results using AI with topic awareness
        """
        from ..agents.skills import AgentSkills
        agent = AgentSkills()
        
        if not search_results:
            fallback_prompt = f"User asked: '{query_text}'. I couldn't find relevant content in the book. Answer politely based on general knowledge, but mention that it's outside the book's scope."
            return agent._generate(fallback_prompt)

        # Combine the retrieved results to form a context
        context = "\n\n".join([result["chunk_text"] for result in search_results[:3]])  # Use top 3 results
        
        # Add topic context if detected
        topic_context = f"\nThis question appears to be about {detected_topic.upper()}." if detected_topic else ""

        system_prompt = f"""
        You are an expert AI tutor for a Physical AI & Robotics textbook.
        Use the following retrieved context from the book to answer the user's question.{topic_context}
        
        Context:
        {context}
        
        If selected text is provided, focus the answer on that specific section.
        """
        
        user_prompt = f"{system_prompt}\n\nQuestion: {query_text}"
        if selected_text:
            user_prompt += f"\n\nSelected Text: {selected_text}"

        return agent._generate(user_prompt)

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