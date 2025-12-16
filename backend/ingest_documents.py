"""
Document Ingestion Script
Ingests all chapter markdown files into the RAG system
"""

import sys
import os
from pathlib import Path

# Add parent directory to path to import modules
sys.path.insert(0, str(Path(__file__).parent))

from sqlalchemy.orm import Session
from src.config.database import SessionLocal, engine
from src.models.document import Base, Document
from src.services.rag_service import RAGService
import uuid

# Create tables
Base.metadata.create_all(bind=engine)

# Chapter files to ingest
CHAPTER_FILES = [
    ("chapter-0-intro.md", 0, "Introduction", "intro"),
    ("chapter-0-5-setup.md", 0.5, "Lab Setup", "setup"),
    ("chapter-1-ros2.md", 1, "ROS 2", "ros2"),
    ("chapter-2-simulation.md", 2, "Simulation", "simulation"),
    ("chapter-3-isaac.md", 3, "NVIDIA Isaac", "isaac"),
    ("chapter-4-vla.md", 4, "VLA Systems", "vla"),
    ("chapter-5-capstone.md", 5, "Capstone Project", "capstone"),
    ("chapter-5-humanoids.md", 5, "Humanoid Robotics", "humanoid"),
    ("chapter-6-conversational.md", 6, "Conversational AI", "conversational"),
    ("chapter-6-rag-implementation.md", 6, "RAG Implementation", "rag"),
    ("chapter-7-why-physical-ai-matters.md", 7, "Why Physical AI Matters", "overview"),
    ("chapter-8-architecture-core-concepts.md", 8, "Architecture", "architecture"),
    ("chapter-9-advanced-topics-physical-ai.md", 9, "Advanced Topics", "advanced"),
    ("chapter-10-conversational-robotics-assessments.md", 10, "Assessments", "assessment"),
    ("chapter-11-hardware-requirements-lab-setup.md", 11, "Hardware Requirements", "hardware"),
    ("chapter-12-cloud-onpremise-deployment.md", 12, "Deployment", "deployment"),
    ("chapter-13-economy-jetson-student-kit.md", 13, "Economy Kit", "hardware"),
    ("chapter-14-future.md", 14, "Future of Physical AI", "future"),
]

def ingest_documents(docs_path: str):
    """
    Ingest all chapter documents into the database and RAG system
    """
    db: Session = SessionLocal()
    rag_service = RAGService()
    
    ingested_count = 0
    failed_count = 0
    
    print("Starting document ingestion...")
    print(f"Looking for documents in: {docs_path}\n")
    
    for filename, chapter_num, title, module in CHAPTER_FILES:
        file_path = os.path.join(docs_path, filename)
        
        if not os.path.exists(file_path):
            print(f"⚠️  File not found: {filename}")
            failed_count += 1
            continue
        
        try:
            # Read the file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Check if document already exists
            existing_doc = db.query(Document).filter(
                Document.title == title,
                Document.chapter_number == chapter_num
            ).first()
            
            if existing_doc:
                print(f"ℹ️  Updating existing: {title} (Chapter {chapter_num})")
                existing_doc.content = content
                existing_doc.module = module
                db.commit()
                document = existing_doc
            else:
                # Create new document
                document = Document(
                    id=str(uuid.uuid4()),
                    title=title,
                    content=content,
                    chapter_number=chapter_num,
                    module=module
                )
                db.add(document)
                db.commit()
                print(f"✓ Created: {title} (Chapter {chapter_num})")
            
            # Index document in RAG system
            print(f"  Indexing into RAG system...")
            success = rag_service.index_document(db, document)
            
            if success:
                print(f"  ✓ Indexed successfully\n")
                ingested_count += 1
            else:
                print(f"  ✗ Failed to index\n")
                failed_count += 1
                
        except Exception as e:
            print(f"✗ Error processing {filename}: {str(e)}\n")
            failed_count += 1
            db.rollback()
    
    db.close()
    
    print("=" * 60)
    print("INGESTION SUMMARY")
    print("=" * 60)
    print(f"Successfully ingested: {ingested_count}")
    print(f"Failed: {failed_count}")
    print(f"Total: {len(CHAPTER_FILES)}")
    print("=" * 60)
    
    return ingested_count, failed_count

if __name__ == "__main__":
    # Default path to docs
    default_path = os.path.join(os.path.dirname(__file__), "..", "frontend", "docs")
    docs_path = sys.argv[1] if len(sys.argv) > 1 else default_path
    
    # Normalize path
    docs_path = os.path.abspath(docs_path)
    
    if not os.path.exists(docs_path):
        print(f"Error: Documents path does not exist: {docs_path}")
        sys.exit(1)
    
    ingested, failed = ingest_documents(docs_path)
    
    sys.exit(0 if failed == 0 else 1)
