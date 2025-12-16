from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    # Database Configuration
    neon_database_url: str

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: Optional[str] = None

    # OpenAI Configuration
    # Gemini Configuration
    gemini_api_key: Optional[str] = None
    model_name: str = "gemini-pro"
    embedding_model_name: str = "models/embedding-001"

    # Application Configuration
    debug: bool = True
    log_level: str = "info"

    # Server Configuration
    host: str = "0.0.0.0"
    port: int = 8000

    class Config:
        env_file = ".env"
        extra = "ignore"

settings = Settings()