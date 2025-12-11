from sqlalchemy import Column, String, Integer, Text, DateTime, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class Embedding(Base):
    __tablename__ = "embeddings"

    id = Column(String, primary_key=True, index=True)
    document_id = Column(String, ForeignKey("documents.id"))  # Reference to the source document
    chunk_text = Column(Text)  # Original text chunk that was embedded
    chunk_index = Column(Integer)  # Position of this chunk within the document
    # Note: The actual vector will be stored in Qdrant, not in the SQL database
    metadata_json = Column(Text)  # Additional metadata about the chunk in JSON format
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    def __repr__(self):
        return f"<Embedding(id={self.id}, document_id={self.document_id}, chunk_index={self.chunk_index})>"