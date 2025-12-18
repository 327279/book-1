from pydantic import BaseModel
from typing import List

class ChatResponse(BaseModel):
    response: str
    sources: List[str]