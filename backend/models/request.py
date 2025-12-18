from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    query: str
    selection_context: Optional[str] = None
    session_id: str