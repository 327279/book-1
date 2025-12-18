# Implementation Plan: BiblioChat Backend

**Branch**: `001-biblio-chat-integration` | **Date**: 2025-12-18 | **Spec**: [specs/001-biblio-chat-integration/spec.md](specs/001-biblio-chat-integration/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a FastAPI-based backend for the BiblioChat application that provides dual-mode RAG chat functionality (selection mode and global mode) with vector storage in Qdrant and chat history persistence in Neon Postgres database. The system will use Cohere's command-r model for AI responses and embed-english-v3.0 for embeddings.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: FastAPI, Cohere, Qdrant-client, SQLAlchemy, asyncpg, python-dotenv
**Storage**: Neon Postgres (chat history), Qdrant (vector embeddings)
**Testing**: pytest (not specified but recommended for backend)
**Target Platform**: Linux server (backend service)
**Project Type**: web (backend service for web application)
**Performance Goals**: <2 seconds response time for AI generation (p95)
**Constraints**: <2 seconds for text selection to answer generation, 1024-dim vector embeddings, 128k token context window
**Scale/Scope**: Single book architecture, hardcoded user_id=1, stateless AI with stateful history

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- Dual-Context Resolution: System must intelligently switch between "Global Retrieval" and "Local Analysis" based on selection_payload presence
- RAG-First Intelligence: All AI responses must be grounded in retrieved content from Qdrant or provided context
- Stateless Intelligence, Stateful History: Vectors are static in Qdrant, conversations persistent in Neon
- Embedding Integrity: Must use embed-english-v3.0 model with 1024 dimensions and cosine distance metric
- Security-First Architecture: API keys must be stored in .env, never exposed to client-side
- Input Limits: Adhere to Cohere's 128k token context window

## Project Structure

### Documentation (this feature)

```text
specs/001-biblio-chat-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # FastAPI application
├── database.py          # SQLAlchemy setup and models
├── vector_setup.py      # Qdrant connection and collection setup
├── ingest.py            # Data ingestion and embedding script
├── models/
│   ├── request.py       # Pydantic request models
│   └── response.py      # Pydantic response models
├── services/
│   ├── chat_service.py  # Core chat logic
│   ├── embedding_service.py # Embedding operations
│   └── history_service.py # Chat history operations
├── requirements.txt     # Dependencies
├── .env                 # Environment variables
├── .gitignore           # Git ignore rules
└── book_source.txt      # Sample book content for testing
```

**Structure Decision**: Selected web application backend structure with FastAPI in the backend directory. Services are organized by functionality (chat, embedding, history) with models separated for request/response validation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |