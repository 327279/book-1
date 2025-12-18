from sqlalchemy.orm import Session
from database import ChatHistory
from datetime import datetime
from typing import List, Optional

class HistoryService:
    def __init__(self):
        pass

    async def save_interaction(self, db: Session, user_message: str, ai_response: str, mode: str, session_id: str, book_id: str = "main_book"):
        """
        Save a chat interaction to the database
        """
        try:
            # Create a new ChatHistory record
            chat_history = ChatHistory(
                user_message=user_message,
                ai_response=ai_response,
                mode=mode,
                session_id=session_id,
                book_id=book_id,
                timestamp=datetime.utcnow()
            )

            # Add to session and commit
            db.add(chat_history)
            db.commit()
            db.refresh(chat_history)

            return chat_history
        except Exception as e:
            db.rollback()
            raise Exception(f"Error saving interaction to history: {str(e)}")

    async def get_session_history(self, db: Session, session_id: str) -> List[ChatHistory]:
        """
        Retrieve all chat interactions for a specific session
        """
        try:
            # Query the database for all records with the given session_id
            history = db.query(ChatHistory).filter(ChatHistory.session_id == session_id).order_by(ChatHistory.timestamp).all()
            return history
        except Exception as e:
            raise Exception(f"Error retrieving session history: {str(e)}")

    async def get_all_sessions(self, db: Session) -> List[str]:
        """
        Get all unique session IDs
        """
        try:
            # Query the database for all unique session IDs
            session_ids = db.query(ChatHistory.session_id).distinct().all()
            return [session_id[0] for session_id in session_ids]
        except Exception as e:
            raise Exception(f"Error retrieving all sessions: {str(e)}")

    async def create_new_session(self, session_id: str = None) -> str:
        """
        Create a new session ID if not provided
        """
        import uuid
        if not session_id:
            session_id = str(uuid.uuid4())
        return session_id

    async def get_session_summary(self, db: Session, session_id: str) -> dict:
        """
        Get a summary of a session including message count and timestamps
        """
        try:
            # Query the database for all records with the given session_id
            history = db.query(ChatHistory).filter(ChatHistory.session_id == session_id).order_by(ChatHistory.timestamp).all()

            if not history:
                return {
                    "session_id": session_id,
                    "message_count": 0,
                    "first_message_time": None,
                    "last_message_time": None
                }

            return {
                "session_id": session_id,
                "message_count": len(history),
                "first_message_time": history[0].timestamp,
                "last_message_time": history[-1].timestamp
            }
        except Exception as e:
            raise Exception(f"Error retrieving session summary: {str(e)}")