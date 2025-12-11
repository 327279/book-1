# Quickstart Guide: Physical AI & Humanoid Robotics: Docusaurus Book & RAG Chatbot

## Prerequisites

- Node.js 18+ for Docusaurus frontend
- Python 3.11+ for FastAPI backend
- Access to Neon Postgres (free tier)
- Access to Qdrant Cloud (free tier)
- Git for version control
- Basic knowledge of ROS 2 and NVIDIA Isaac (for understanding the content)

## Setting Up the Development Environment

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up the Frontend (Docusaurus)

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Start the development server
npm start
```

The Docusaurus site will be available at `http://localhost:3000`

### 3. Set Up the Backend (FastAPI)

```bash
# Navigate to backend directory
cd backend

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your Neon and Qdrant credentials

# Start the development server
uvicorn src.api.main:app --reload --port 8000
```

The backend API will be available at `http://localhost:8000`

### 4. Configure Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
NEON_DATABASE_URL=your_neon_database_url
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key  # If using OpenAI models
MODEL_NAME=text-embedding-ada-002  # Or your preferred embedding model
DEBUG=true
```

## Running the Full Application

### 1. Start the Backend

```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn src.api.main:app --reload --port 8000
```

### 2. Start the Frontend

```bash
cd frontend
npm start
```

The application will be available at `http://localhost:3000` with the RAG chatbot integrated into the pages.

## Adding New Content

### 1. Create a New Chapter

Create a new markdown file in the `frontend/docs` directory:

```markdown
---
title: Chapter X: Your Chapter Title
sidebar_position: X
---

# Your Chapter Title

Your chapter content here...

## Section 1

Content for section 1...

## Section 2

Content for section 2...
```

### 2. Index the Content in the RAG System

After adding new content, you need to index it:

```bash
cd backend
source venv/bin/activate
python -m src.services.document_service index_new_documents
```

## Testing the RAG Functionality

### 1. Unit Tests

```bash
# Backend tests
cd backend
python -m pytest tests/unit/

# Frontend tests
cd frontend
npm test
```

### 2. Integration Tests

```bash
# Test the RAG functionality
cd backend
python -m pytest tests/integration/

# Test end-to-end functionality
npm run test:e2e  # In frontend directory
```

## Deployment

### 1. Deploy Frontend to GitHub Pages

```bash
cd frontend
GIT_USER=<your-github-username> npm run deploy
```

### 2. Deploy Backend to Cloud Provider

The backend can be deployed to any cloud provider that supports Python applications (e.g., Heroku, Render, AWS, GCP).

For Render deployment, use the provided `render.yaml` file.

## Troubleshooting

### Common Issues

1. **RAG queries returning no results**: Make sure the documents have been properly indexed in Qdrant.

2. **Frontend can't connect to backend**: Check that the backend is running and that the API endpoint is correctly configured in the frontend.

3. **Embedding errors**: Verify that your embedding model API key is valid and has sufficient quota.

### Useful Commands

```bash
# Check backend health
curl http://localhost:8000/api/v1/health

# List all indexed documents
curl http://localhost:8000/api/v1/documents

# Test a query
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "your question here"}'
```