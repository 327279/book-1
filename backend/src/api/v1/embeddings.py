from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List, Dict, Any
from ...config.database import get_db
from ...services.embedding_service import EmbeddingService
from ...services.document_service import DocumentService

router = APIRouter(prefix="/api/v1/embeddings", tags=["embeddings"])

@router.post("/index-document/{document_id}")
def index_document(document_id: str, db: Session = Depends(get_db)):
    """
    Index a document's content into the vector store
    """
    document = DocumentService.get_document_by_id(db, document_id)
    if not document:
        raise HTTPException(status_code=404, detail="Document not found")

    embedding_service = EmbeddingService()
    success = embedding_service.index_document_content(document)

    if success:
        return {"message": f"Successfully indexed document {document_id}"}
    else:
        raise HTTPException(status_code=500, detail="Failed to index document")

@router.post("/search")
def search_embeddings(query: str, limit: int = 5):
    """
    Search for similar content in the vector store
    """
    embedding_service = EmbeddingService()
    query_embedding = embedding_service.generate_embeddings(query)
    results = embedding_service.search_similar(query_embedding, limit=limit)

    return {
        "query": query,
        "results": results
    }

@router.delete("/document/{document_id}")
def delete_document_embeddings(document_id: str):
    """
    Delete all embeddings associated with a specific document
    """
    embedding_service = EmbeddingService()
    success = embedding_service.delete_document_embeddings(document_id)

    if success:
        return {"message": f"Successfully deleted embeddings for document {document_id}"}
    else:
        raise HTTPException(status_code=500, detail="Failed to delete document embeddings")