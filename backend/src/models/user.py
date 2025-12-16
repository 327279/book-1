from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime

class User(BaseModel):
    """
    User model for authentication and profile management
    """
    id: str
    email: EmailStr
    name: Optional[str] = None
    hashed_password: str
    created_at: datetime = datetime.utcnow()
    is_active: bool = True
    
    # Profile fields
    software_bg: Optional[str] = "General"
    hardware_bg: Optional[str] = "General"

    class Config:
        from_attributes = True
