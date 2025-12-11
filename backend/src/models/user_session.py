from sqlalchemy import Column, String, DateTime, Boolean
from sqlalchemy.sql import func
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class UserSession(Base):
    __tablename__ = "user_sessions"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, index=True)  # Optional identifier for the user
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    page_url = Column(String)  # URL of the page where the session started
    active = Column(Boolean, default=True)  # Boolean indicating if the session is currently active

    def __repr__(self):
        return f"<UserSession(id={self.id}, user_id={self.user_id}, active={self.active})>"