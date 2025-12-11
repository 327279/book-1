from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
from ...config.database import get_db
from ...models.document import Document
from ...services.document_service import DocumentService

router = APIRouter(prefix="/api/v1/documents", tags=["documents"])

@router.get("/", response_model=List[dict])
def get_all_documents(db: Session = Depends(get_db)):
    """
    Retrieve all documents (chapters) from the database
    """
    documents = DocumentService.get_all_documents(db)
    return [
        {
            "id": doc.id,
            "title": doc.title,
            "chapter_number": doc.chapter_number,
            "module": doc.module,
            "created_at": doc.created_at.isoformat() if doc.created_at else None
        }
        for doc in documents
    ]

@router.get("/{document_id}", response_model=dict)
def get_document(document_id: str, db: Session = Depends(get_db)):
    """
    Retrieve a specific document by ID
    """
    document = DocumentService.get_document_by_id(db, document_id)
    if not document:
        raise HTTPException(status_code=404, detail="Document not found")

    return {
        "id": document.id,
        "title": document.title,
        "content": document.content,
        "chapter_number": document.chapter_number,
        "module": document.module,
        "created_at": document.created_at.isoformat() if document.created_at else None,
        "updated_at": document.updated_at.isoformat() if document.updated_at else None
    }

@router.post("/", response_model=dict)
def create_document(document_data: dict, db: Session = Depends(get_db)):
    """
    Create a new document
    """
    # Generate ID if not provided
    if "id" not in document_data:
        import uuid
        document_data["id"] = str(uuid.uuid4())

    # Validate required fields
    required_fields = ["title", "content", "chapter_number", "module"]
    for field in required_fields:
        if field not in document_data:
            raise HTTPException(status_code=400, detail=f"Missing required field: {field}")

    # Validate chapter_number is between 1-6
    if not (1 <= document_data["chapter_number"] <= 6):
        raise HTTPException(status_code=400, detail="Chapter number must be between 1 and 6")

    document = DocumentService.create_document(db, document_data)
    return {
        "id": document.id,
        "title": document.title,
        "chapter_number": document.chapter_number,
        "module": document.module
    }

@router.put("/{document_id}", response_model=dict)
def update_document(document_id: str, document_data: dict, db: Session = Depends(get_db)):
    """
    Update a document by ID
    """
    # Remove ID from update data to prevent changing the document ID
    if "id" in document_data:
        del document_data["id"]

    document = DocumentService.update_document(db, document_id, document_data)
    if not document:
        raise HTTPException(status_code=404, detail="Document not found")

    return {
        "id": document.id,
        "title": document.title,
        "chapter_number": document.chapter_number,
        "module": document.module
    }

@router.delete("/{document_id}")
def delete_document(document_id: str, db: Session = Depends(get_db)):
    """
    Delete a document by ID
    """
    success = DocumentService.delete_document(db, document_id)
    if not success:
        raise HTTPException(status_code=404, detail="Document not found")

    return {"message": "Document deleted successfully"}