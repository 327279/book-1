from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, EmailStr
from typing import Optional
from passlib.context import CryptContext
import jwt
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
from ...config.database import get_db
from ...models.user import User
from ...config.settings import settings
import uuid

router = APIRouter(prefix="/api/auth", tags=["auth"])

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT settings
SECRET_KEY = settings.openai_api_key or "better-auth-secret-key-change-in-production"
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24 * 7  # 7 days

class SignUpRequest(BaseModel):
    email: EmailStr
    password: str
    name: Optional[str] = None

class SignInRequest(BaseModel):
    email: EmailStr
    password: str

class UserProfileRequest(BaseModel):
    user_id: str
    email: str
    software_bg: str
    hardware_bg: str

class TokenResponse(BaseModel):
    user: dict
    token: str

def create_access_token(data: dict):
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

# In-memory user store (for demo - in production use database)
users_db = {}
user_profiles = {}

@router.post("/sign-up/email")
async def sign_up_email(request: SignUpRequest):
    """
    Better-Auth compatible email signup endpoint
    """
    if request.email in users_db:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    user_id = str(uuid.uuid4())
    hashed_password = get_password_hash(request.password)
    
    user = {
        "id": user_id,
        "email": request.email,
        "name": request.name or request.email.split('@')[0],
        "createdAt": datetime.utcnow().isoformat()
    }
    
    users_db[request.email] = {
        **user,
        "password": hashed_password
    }
    
    token = create_access_token({"sub": request.email, "user_id": user_id})
    
    return {
        "user": user,
        "token": token
    }

@router.post("/sign-in/email")
async def sign_in_email(request: SignInRequest):
    """
    Better-Auth compatible email signin endpoint
    """
    if request.email not in users_db:
        raise HTTPException(status_code=401, detail="Invalid email or password")
    
    stored_user = users_db[request.email]
    
    if not verify_password(request.password, stored_user["password"]):
        raise HTTPException(status_code=401, detail="Invalid email or password")
    
    user = {
        "id": stored_user["id"],
        "email": stored_user["email"],
        "name": stored_user["name"],
        "createdAt": stored_user["createdAt"]
    }
    
    token = create_access_token({"sub": request.email, "user_id": stored_user["id"]})
    
    return {
        "user": user,
        "token": token
    }

@router.post("/sign-out")
async def sign_out():
    """
    Better-Auth compatible signout endpoint
    """
    # In a real implementation, invalidate the token
    return {"success": True}

@router.get("/session")
async def get_session():
    """
    Better-Auth compatible session check endpoint
    """
    # In production, verify JWT from Authorization header
    return {"user": None}

@router.post("/users/profile")
async def save_user_profile(profile: UserProfileRequest):
    """
    Save user profile for personalization
    """
    user_profiles[profile.user_id] = {
        "email": profile.email,
        "software_bg": profile.software_bg,
        "hardware_bg": profile.hardware_bg
    }
    return {"success": True, "profile": user_profiles[profile.user_id]}

@router.get("/users/profile/{user_id}")
async def get_user_profile(user_id: str):
    """
    Get user profile for personalization
    """
    if user_id not in user_profiles:
        return {"software_bg": "General", "hardware_bg": "General"}
    return user_profiles[user_id]
