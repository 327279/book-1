# Quickstart: BiblioChat Backend

**Date**: 2025-12-18
**Feature**: BiblioChat Backend
**Branch**: 001-biblio-chat-integration

## Overview

This guide provides step-by-step instructions to set up, configure, and run the BiblioChat backend service locally.

## Prerequisites

- Python 3.10 or higher
- pip package manager
- Git (for version control)
- Access to the following services:
  - Cohere API account with command-r and embed-english-v3.0 access
  - Qdrant Cloud account
  - Neon Postgres account

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
cd backend  # Navigate to the backend directory
```

### 2. Set up Python Virtual Environment

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

If requirements.txt doesn't exist, install the following packages:

```bash
pip install fastapi uvicorn[standard] pydantic cohere qdrant-client sqlalchemy asyncpg python-dotenv
```

### 4. Configure Environment Variables

Create a `.env` file in the backend directory with the following content:

```env
# Cohere Configuration
COHERE_API_KEY=7ys16G9aChXiWXxmhWuxL9N42pFdEe8bSvKjVRl7

# Qdrant Configuration
QDRANT_HOST=https://6cfdd60d-3649-4547-acb0-7aa88ce8444d.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.info0CDXMgdhMcapDEZQkbAZFBXBWIEooEkipqT64co
QDRANT_COLLECTION_NAME=biblio_chat

# Neon Postgres Configuration
DATABASE_URL=postgresql://neondb_owner:npg_FxR5tiXI3dkv@ep-muddy-frog-ah6jzfcs-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
```

### 5. Initialize the Database

Run the database initialization script to create tables:

```bash
python init_db.py
```

If the script doesn't exist, create it with:

```python
# init_db.py
from database import engine, Base

async def init_db():
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    await engine.dispose()

if __name__ == "__main__":
    import asyncio
    asyncio.run(init_db())
    print("Database tables created successfully!")
```

### 6. Initialize the Vector Database (Qdrant)

Run the vector setup script to create the collection:

```bash
python vector_setup.py
```

### 7. Prepare Sample Book Content

Create a `book_source.txt` file with sample book content for testing:

```bash
# Create a sample book file
echo "This is a sample book content for testing the BiblioChat backend. The AI assistant will be able to answer questions about this text content using the RAG system. You can replace this with actual book content for real usage." > book_source.txt
```

### 8. Ingest Book Content into Vector Database

Run the ingestion script to process the book content and store it in Qdrant:

```bash
python ingest.py
```

## Running the Application

### Development Mode

```bash
# Run the FastAPI application in development mode
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Production Mode

```bash
# Run the FastAPI application in production mode
uvicorn main:app --host 0.0.0.0 --port 8000
```

## API Endpoints

Once the application is running, you can access the API documentation at:
- Swagger UI: http://127.0.0.1:8000/docs
- ReDoc: http://127.0.0.1:8000/redoc

### Main Endpoint

- **POST** `/api/chat`
  - Request body: `{"query": "your question", "selection_context": "selected text (or null)", "session_id": "unique_session_id"}`
  - Response: `{"response": "AI response", "sources": ["source1", "source2"]}`

## Testing the Application

### 1. Test Global Mode (No Selection)

```bash
curl -X POST "http://127.0.0.1:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is this text about?",
    "selection_context": null,
    "session_id": "test-session-1"
  }'
```

### 2. Test Selection Mode (With Context)

```bash
curl -X POST "http://127.0.0.1:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept",
    "selection_context": "This is a sample book content for testing the BiblioChat backend",
    "session_id": "test-session-1"
  }'
```

## Troubleshooting

### Common Issues

1. **Environment Variables Not Loaded**
   - Ensure `.env` file is in the correct directory
   - Verify that `python-dotenv` is installed and being used

2. **Database Connection Issues**
   - Check that the Neon Postgres connection string is correct
   - Verify that your IP address has access to the Neon database

3. **Qdrant Connection Issues**
   - Ensure QDRANT_HOST is properly formatted
   - Verify that QDRANT_API_KEY is correct

4. **Cohere API Issues**
   - Check that COHERE_API_KEY is valid
   - Verify that your Cohere account has access to command-r and embed-english-v3.0

### Verifying Setup

1. Check that the application starts without errors
2. Access the Swagger UI at http://127.0.0.1:8000/docs
3. Verify that the database tables were created
4. Confirm that the Qdrant collection exists and has content after ingestion

## Next Steps

1. Integrate with the frontend application
2. Add authentication layer
3. Implement additional book sources
4. Add monitoring and logging