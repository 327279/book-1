from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import Optional
from pydantic import BaseModel
from ...config.database import get_db
from ...services.rag_service import RAGService

router = APIRouter(prefix="/api/v1/queries", tags=["queries"])

class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    user_id: Optional[str] = None

@router.post("/")
def create_query(
    request: QueryRequest,
    db: Session = Depends(get_db)
):
    """
    Submit a query to the RAG system
    """
    if not request.query or not request.query.strip():
        raise HTTPException(status_code=400, detail="Query text is required")

    rag_service = RAGService()
    # Map user_id to session_id for internal service
    query_result = rag_service.process_query(db, request.query, request.selected_text, request.user_id)

    # Safely parse source documents
    try:
        import json
        source_docs = json.loads(query_result.source_documents_json)
    except:
        source_docs = []

    return {
        "response": query_result.response_text,
        "sources": [
            {
                "document_id": doc_id,
                "title": f"Document {doc_id}", 
                "chapter": 1 # Placeholder, strictly matching frontend expectation
            }
            for doc_id in source_docs
        ],
        "confidence": query_result.confidence_score,
        "query_id": query_result.id
    }

@router.get("/health")
def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "message": "RAG API is running"}