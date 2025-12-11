from sqlalchemy.orm import Session
from typing import List, Optional
from ..models.document import Document
from ..models.code_example import CodeExample

class DocumentService:
    @staticmethod
    def create_document(db: Session, document_data: dict) -> Document:
        """
        Create a new document in the database
        """
        document = Document(**document_data)
        db.add(document)
        db.commit()
        db.refresh(document)
        return document

    @staticmethod
    def get_document_by_id(db: Session, document_id: str) -> Optional[Document]:
        """
        Retrieve a document by its ID
        """
        return db.query(Document).filter(Document.id == document_id).first()

    @staticmethod
    def get_document_by_chapter_number(db: Session, chapter_number: int) -> Optional[Document]:
        """
        Retrieve a document by its chapter number
        """
        return db.query(Document).filter(Document.chapter_number == chapter_number).first()

    @staticmethod
    def get_all_documents(db: Session) -> List[Document]:
        """
        Retrieve all documents
        """
        return db.query(Document).all()

    @staticmethod
    def update_document(db: Session, document_id: str, document_data: dict) -> Optional[Document]:
        """
        Update a document by its ID
        """
        document = db.query(Document).filter(Document.id == document_id).first()
        if document:
            for key, value in document_data.items():
                setattr(document, key, value)
            db.commit()
            db.refresh(document)
        return document

    @staticmethod
    def delete_document(db: Session, document_id: str) -> bool:
        """
        Delete a document by its ID
        """
        document = db.query(Document).filter(Document.id == document_id).first()
        if document:
            db.delete(document)
            db.commit()
            return True
        return False

    @staticmethod
    def get_code_examples_for_document(db: Session, document_id: str) -> List[CodeExample]:
        """
        Get all code examples associated with a specific document
        """
        return db.query(CodeExample).filter(CodeExample.document_id == document_id).all()

    @staticmethod
    def index_document_content(db: Session, document: Document) -> bool:
        """
        Process and index document content for the RAG system
        This involves chunking the content and storing embeddings in Qdrant
        """
        from .rag_service import RAGService
        rag_service = RAGService()
        return rag_service.index_document(db, document)