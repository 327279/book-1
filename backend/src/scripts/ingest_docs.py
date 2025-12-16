import os
import sys
import glob
from sqlalchemy.orm import Session
from src.config.database import SessionLocal, engine
from src.services.rag_service import RAGService
from src.models.document import Document
from src.models.base import Base

def ingest_documents():
    """
    Ingests all markdown files from frontend/docs into the RAG system.
    """
    # ensure tables exist
    Base.metadata.create_all(bind=engine)
    
    db: Session = SessionLocal()
    rag = RAGService()
    
    docs_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../frontend/docs"))
    md_files = glob.glob(os.path.join(docs_dir, "**/*.md"), recursive=True)
    
    print(f"Found {len(md_files)} markdown files in {docs_dir}")
    
    for file_path in md_files:
        print(f"Processing {file_path}...")
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
            
            # Extract basic metadata (naive)
            filename = os.path.basename(file_path)
            chapter_num = 0
            if "chapter-" in filename:
                try:
                    parts = filename.split('-')
                    chapter_num = float(parts[1]) if parts[1].replace('.','',1).isdigit() else 0
                except:
                    pass
            
            # Check if doc exists
            existing = db.query(Document).filter(Document.source_path == file_path).first()
            if existing:
                print(f"  - Updating existing document: {filename}")
                existing.content = content
                doc = existing
            else:
                print(f"  - Creating new document: {filename}")
                doc = Document(
                    title=filename.replace(".md", "").replace("-", " ").title(),
                    content=content,
                    source_path=file_path,
                    chapter_number=chapter_num,
                    module="Core"
                )
                db.add(doc)
            
            db.commit()
            db.refresh(doc)
            
            # Index in Qdrant
            success = rag.index_document(db, doc)
            if success:
                print("  - Indexed successfully")
            else:
                print("  - Indexing FAILED")
                
        except Exception as e:
            print(f"Error processing {file_path}: {e}")

    db.close()
    print("Ingestion complete.")

if __name__ == "__main__":
    # Add backend to path
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))
    ingest_documents()
