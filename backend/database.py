from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy import Column, Integer, String, Text, DateTime
from datetime import datetime
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Database URL from environment
DATABASE_URL = os.getenv("DATABASE_URL")

# Create async engine with async driver (using asyncpg)
# Replace postgresql:// with postgresql+asyncpg:// for async support
if DATABASE_URL and DATABASE_URL.startswith("postgresql://"):
    ASYNC_DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
    SYNC_DATABASE_URL = DATABASE_URL  # Keep original for sync operations
elif DATABASE_URL and DATABASE_URL.startswith("sqlite://"):
    ASYNC_DATABASE_URL = DATABASE_URL
    SYNC_DATABASE_URL = DATABASE_URL
else:
    ASYNC_DATABASE_URL = DATABASE_URL
    SYNC_DATABASE_URL = DATABASE_URL

# Create async engine
async_engine = create_async_engine(ASYNC_DATABASE_URL)

# Create sync engine for compatibility with existing code
sync_engine = create_engine(SYNC_DATABASE_URL)

# Create async session
AsyncSessionLocal = async_sessionmaker(autoflush=False, bind=async_engine, expire_on_commit=False)

# Create sync session for compatibility with existing code
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=sync_engine)

# Base class for models
Base = declarative_base()


# ChatHistory Model - follows data-model.md specification
class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, index=True)
    user_message = Column(Text, nullable=False)
    ai_response = Column(Text, nullable=False)
    mode = Column(String(20), nullable=False)  # 'selection' or 'global'
    session_id = Column(String(255), nullable=False)
    book_id = Column(String(255), nullable=False, default="main_book")
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow)


# Dependency to get database session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Create tables asynchronously
async def init_db():
    async with async_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

# Synchronous version for compatibility
def init_db_sync():
    Base.metadata.create_all(bind=sync_engine)


if __name__ == "__main__":
    init_db_sync()