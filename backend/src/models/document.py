from sqlalchemy import Column, Integer, String, Text, DateTime
from sqlalchemy.sql import func
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class Document(Base):
    __tablename__ = "documents"

    id = Column(String, primary_key=True, index=True)
    title = Column(String, index=True)
    content = Column(Text)
    chapter_number = Column(Integer)  # 1-6 for the 6 chapters
    module = Column(String)  # e.g., "ROS2", "Simulation", "Isaac", "VLA", "Capstone", "RAG"
    source_url = Column(String)  # URL to the original document source
    content_type = Column(String)  # Type of content (e.g., "text/markdown")
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    version = Column(String, default="1.0")

    def __repr__(self):
        return f"<Document(id={self.id}, title={self.title}, chapter_number={self.chapter_number})>"