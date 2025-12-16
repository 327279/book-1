# Physical AI & Humanoid Robotics: Interactive Textbook

An advanced, interactive textbook on Physical AI and Humanoid Robotics featuring an embedded RAG-powered chatbot. Built with Docusaurus and FastAPI.

🌐 **Live Site**: [https://book-1-eight.vercel.app/](https://book-1-eight.vercel.app/)

## 📚 Features

- **18 Comprehensive Chapters** covering ROS 2, Simulation, NVIDIA Isaac, VLA systems, and more
- **Intelligent RAG Chatbot** with topic-aware query processing
- **Interactive Code Examples** for ROS 2, Gazebo, Isaac, and VLA
- **Modern UI** with dark mode, syntax highlighting, and responsive design
- **Multi-language Support** including Urdu translation
- **User Authentication** with Better-Auth integration

## 🏗️ Project Structure

```
book/
├── frontend/          # Docusaurus application
│   ├── docs/         # 18 chapter markdown files
│   ├── src/          # React components (ChatBot, etc.)
│   └── static/       # Images and assets
├── backend/          # FastAPI application
│   ├── src/
│   │   ├── api/      # REST API routes
│   │   ├── models/   # SQLAlchemy models
│   │   ├── services/ # Business logic (RAG, embeddings)
│   │   └── agents/   # AI agent skills
│   └── test_rag_system.py  # Comprehensive tests
└── specs/            # Project specifications
```

## 🚀 Quick Start

### Prerequisites
- Node.js 18+
- Python 3.9+
- OpenAI API key
- Qdrant instance (local or cloud)

### Frontend Setup
```bash
cd frontend
npm install
npm run start  # Development server at http://localhost:3000
npm run build  # Production build
```

### Backend Setup
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt

# Configure environment variables
cp .env.example .env
# Edit .env with your API keys

# Start server
uvicorn src.api.main:app --reload  # At http://localhost:8000
```

## 🧪 Testing

### RAG System Tests
```bash
cd backend
python test_rag_system.py http://localhost:8000
```

Tests 28 queries across 7 topics (ROS 2, simulation, Isaac, VLA, capstone, conversational, humanoid) and generates a detailed accuracy report.

## 📖 Chapter Overview

1. **Chapter 0** - Introduction to Physical AI
2. **Chapter 0.5** - Lab Setup & Prerequisites
3. **Chapter 1** - ROS 2 Architecture (Nodes, Topics, Services, URDF)
4. **Chapter 2** - Simulation (Gazebo & Unity)
5. **Chapter 3** - NVIDIA Isaac (Sim, ROS, Nav2)
6. **Chapter 4** - VLA Systems (Voice-Language-Action)
7. **Chapter 5** - Capstone Project
8. **Chapter 5 (Alt)** - Humanoid Robotics
9. **Chapter 6** - Conversational AI
10. **Chapter 6 (Alt)** - RAG Implementation
11. **Chapter 7** - Why Physical AI Matters
12. **Chapter 8** - Architecture & Core Concepts
13. **Chapter 9** - Advanced Topics
14. **Chapter 10** - Assessments
15. **Chapter 11** - Hardware Requirements
16. **Chapter 12** - Cloud & On-Premise Deployment
17. **Chapter 13** - Economy Jetson Student Kit
18. **Chapter 14** - Future of Physical AI

## 🤖 RAG System

The embedded chatbot uses a Retrieval-Augmented Generation (RAG) system with:

- **Topic Detection** - Automatically classifies queries into 7 categories
- **Vector Search** - Qdrant with OpenAI embeddings (1536 dimensions)
- **Context-Aware Responses** - Uses detected topics to filter and enhance results
- **Source Citation** - Tracks and returns source documents
- **Confidence Scoring** - 90% for topic-specific, 85% for general queries

### API Endpoints
- `POST /api/v1/queries/` - Submit RAG queries
- `GET /api/v1/documents/` - List indexed documents
- `POST /api/v1/embeddings/` - Generate embeddings

## 🛠️ Technology Stack

### Frontend
- Docusaurus 3.1.0
- React 18
- MDX for content
- Better-Auth for authentication
- OpenAI SDK for chat

### Backend
- FastAPI
- SQLAlchemy (SQLite/Postgres)
- Qdrant (vector database)
- OpenAI API (embeddings & generation)
- JWT authentication

## 📊 Project Status

✅ **95% Complete** - Feature complete, ready for deployment verification

- ✅ All 18 chapters implemented
- ✅ RAG system with topic detection
- ✅ Frontend builds successfully
- ✅ Backend fully functional
- ✅ Test suite created
- 🔄 Awaiting production backend deployment

## 🚢 Deployment

### Frontend (Vercel)
Automatically deploys on push to main branch. Current deployment: https://book-1-eight.vercel.app/

### Backend
Requires:
- Environment variables (OPENAI_API_KEY, DATABASE_URL, QDRANT_URL)
- Neon Postgres database
- Qdrant Cloud instance
- CORS configuration for frontend domain

## 📝 License

Educational project for Physical AI & Humanoid Robotics course.

## 🤝 Contributing

This project was built as an educational resource. For suggestions or improvements, please open an issue.

---

**Built with** ❤️ **for the Physical AI community**
