from sqlalchemy import Column, String, Text, DateTime, Integer
from sqlalchemy.sql import func
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class Query(Base):
    __tablename__ = "queries"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, index=True)  # Optional for anonymous usage
    query_text = Column(Text)  # Original text of the user's query
    selected_text = Column(Text)  # Text selected by user when making the query (optional)
    context_json = Column(Text)  # Additional context provided with the query in JSON format
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    session_id = Column(String, index=True)  # Identifier for the chat session

    def __repr__(self):
        return f"<Query(id={self.id}, user_id={self.user_id}, session_id={self.session_id})>"