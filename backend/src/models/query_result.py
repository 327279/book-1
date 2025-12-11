from sqlalchemy import Column, String, Text, DateTime, Float, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class QueryResult(Base):
    __tablename__ = "query_results"

    id = Column(String, primary_key=True, index=True)
    query_id = Column(String, ForeignKey("queries.id"))  # Reference to the original query
    response_text = Column(Text)  # AI-generated response to the query
    source_documents_json = Column(Text)  # List of document IDs used to generate the response in JSON format
    confidence_score = Column(Float)  # Confidence level of the response (0-1)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    retrieved_chunks_json = Column(Text)  # List of chunk IDs used to generate the response in JSON format

    def __repr__(self):
        return f"<QueryResult(id={self.id}, query_id={self.query_id}, confidence_score={self.confidence_score})>"