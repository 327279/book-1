# BiblioChat Backend

AI-Native Integrated RAG Book Reader Backend - A FastAPI application that provides dual-mode RAG chat functionality for book content analysis.

## Project Structure

```
backend/
├── main.py                 # FastAPI application entry point
├── database.py             # SQLAlchemy database models and setup
├── requirements.txt        # Python dependencies
├── .env                   # Environment variables (not committed)
├── .gitignore             # Git ignore rules
├── init_db.py             # Database initialization script
├── README.md              # Project documentation
├── models/                # Pydantic models
│   ├── request.py         # Request models (ChatRequest)
│   └── response.py        # Response models (ChatResponse)
├── services/              # Business logic services
│   ├── chat_service.py    # Main chat processing logic
│   ├── embedding_service.py # Embedding generation
│   └── history_service.py # Chat history management
├── vector_setup.py        # Qdrant vector database setup
└── ingest.py             # Book content ingestion pipeline
```

## Features

- **Dual-Mode Chat Logic**:
  - Selection Mode: Direct processing of selected text context
  - Global Mode: Vector search through book content using Qdrant
- **Chat History Persistence**: Session-based conversation history in Neon Postgres
- **Vector Storage**: Book content stored in Qdrant vector database
- **Cohere AI Integration**: Using command-r model for responses and embed-english-v3.0 for embeddings

## Setup

1. Install dependencies: `pip install -r requirements.txt`
2. Set up environment variables in `.env` file
3. Initialize database: `python init_db.py`
4. Start the server: `uvicorn main:app --reload`

## Environment Variables

The following environment variables are required:

- `DATABASE_URL`: Neon Postgres connection string
- `COHERE_API_KEY`: Cohere API key
- `QDRANT_HOST`: Qdrant cluster URL
- `QDRANT_API_KEY`: Qdrant API key
- `QDRANT_COLLECTION_NAME`: Qdrant collection name (default: biblio_chat)

## API Endpoints

- `POST /api/chat` - Process chat requests with dual-mode logic
- `GET /api/chat/history/{session_id}` - Retrieve chat history for a session
- `GET /api/health` - Health check endpoint

## Architecture

- **FastAPI**: Web framework with automatic API documentation
- **SQLAlchemy**: Database ORM with asyncpg for Postgres
- **Cohere**: AI models for chat and embeddings
- **Qdrant**: Vector database for RAG functionality
- **Pydantic**: Request/response validation