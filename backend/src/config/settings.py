from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    # Database Configuration
    neon_database_url: str

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: Optional[str] = None

    # OpenAI Configuration
    openai_api_key: Optional[str] = None
    model_name: str = "gpt-4o-mini"
    embedding_model_name: str = "text-embedding-3-small"

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