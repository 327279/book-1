from sqlalchemy import Column, String, Text, DateTime, Integer, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class CodeExample(Base):
    __tablename__ = "code_examples"

    id = Column(String, primary_key=True, index=True)
    document_id = Column(String, ForeignKey("documents.id"))  # Reference to the containing document
    title = Column(String)  # Brief description of the code example
    language = Column(String)  # Programming language (e.g., "python", "bash", "yaml")
    code = Column(Text)  # The actual code content
    description = Column(Text)  # Explanation of what the code does
    execution_context = Column(Text)  # Environment requirements for running the code
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<CodeExample(id={self.id}, document_id={self.document_id}, language={self.language})>"