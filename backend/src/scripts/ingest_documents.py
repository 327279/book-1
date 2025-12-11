#!/usr/bin/env python3
"""
Script to ingest book content from markdown files into the database and index it in Qdrant
"""

import os
import sys
import uuid
from pathlib import Path

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from ..config.database import Base, get_db
from ..models.document import Document
from ..services.document_service import DocumentService
from ..config.settings import settings
from typing import List


def read_markdown_files(docs_path: str) -> List[dict]:
    """
    Read all markdown files from the docs directory and extract content
    """
    docs_dir = Path(docs_path)
    documents = []

    for md_file in docs_dir.glob("*.md"):
        if md_file.name.startswith("chapter-"):
            # Extract chapter number from filename (e.g., chapter-1-ros2.md -> 1)
            try:
                chapter_number = int(md_file.name.split("-")[1])
                title = md_file.name.replace("chapter-", "").replace(".md", "").replace("-", " ").title()

                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                documents.append({
                    "id": str(uuid.uuid4()),
                    "title": title,
                    "content": content,
                    "chapter_number": chapter_number,
                    "module": "Robotics Textbook",
                    "source_url": f"/docs/{md_file.name}",
                    "content_type": "text/markdown"
                })
            except (ValueError, IndexError):
                print(f"Could not parse chapter number from {md_file.name}")
                continue

    return documents


def ingest_documents(db: Session, docs_path: str):
    """
    Ingest documents from markdown files into the database and index them
    """
    print("Reading markdown files...")
    documents_data = read_markdown_files(docs_path)

    print(f"Found {len(documents_data)} documents to ingest")

    for doc_data in documents_data:
        print(f"Processing chapter {doc_data['chapter_number']}: {doc_data['title']}")

        # Check if document already exists
        existing_doc = DocumentService.get_document_by_chapter_number(db, doc_data['chapter_number'])

        if existing_doc:
            print(f"  Document for chapter {doc_data['chapter_number']} already exists, updating...")
            updated_doc = DocumentService.update_document(db, existing_doc.id, doc_data)
        else:
            print(f"  Creating new document for chapter {doc_data['chapter_number']}...")
            updated_doc = DocumentService.create_document(db, doc_data)

        # Index the document content in Qdrant
        print(f"  Indexing content in vector store...")
        success = DocumentService.index_document_content(db, updated_doc)

        if success:
            print(f"    Successfully indexed chapter {doc_data['chapter_number']}")
        else:
            print(f"    Failed to index chapter {doc_data['chapter_number']}")

    print("Document ingestion completed!")


if __name__ == "__main__":
    # Create a fresh engine for this script
    fresh_engine = create_engine(settings.neon_database_url)
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=fresh_engine)

    # Create all database tables first
    print("Creating database tables...")
    Base.metadata.create_all(bind=fresh_engine)
    print("Database tables created successfully!")

    # Get the path to the frontend docs directory
    # Assuming this script is run from the backend directory
    frontend_docs_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "frontend", "docs")

    if not os.path.exists(frontend_docs_path):
        print(f"Error: Frontend docs directory not found at {frontend_docs_path}")
        sys.exit(1)

    print(f"Ingesting documents from: {frontend_docs_path}")

    # Get database session (after tables are created)
    db = SessionLocal()

    try:
        ingest_documents(db, frontend_docs_path)
    finally:
        db.close()