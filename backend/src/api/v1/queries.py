from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import Optional
from ...config.database import get_db
from ...services.rag_service import RAGService

router = APIRouter(prefix="/api/v1/queries", tags=["queries"])

@router.post("/")
def create_query(
    query: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None,
    db: Session = Depends(get_db)
):
    """
    Submit a query to the RAG system
    """
    if not query or not query.strip():
        raise HTTPException(status_code=400, detail="Query text is required")

    rag_service = RAGService()
    query_result = rag_service.process_query(db, query, selected_text, session_id)

    return {
        "response": query_result.response_text,
        "sources": [
            {
                "document_id": doc_id,
                "title": "Document Title",  # In a real implementation, we'd fetch the title from the DB
                "chapter_number": 0  # In a real implementation, we'd fetch this from the DB
            }
            for doc_id in eval(query_result.source_documents_json)  # Note: In production, use json.loads instead of eval
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