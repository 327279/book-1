# Implementation Plan: Physical AI & Humanoid Robotics: Docusaurus Book & RAG Chatbot

**Branch**: `001-docusaurus-book-rag` | **Date**: 2025-12-10 | **Spec**: [link to spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-book-rag/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive interactive textbook for Physical AI & Humanoid Robotics using Docusaurus as the documentation framework and a RAG system built with FastAPI, Neon Postgres, and Qdrant. The system will include 6 chapters covering ROS 2, simulation environments, NVIDIA Isaac, VLA systems, a capstone project, and RAG implementation. The RAG chatbot will be integrated directly into the Docusaurus pages to allow users to query selected text from the book content.

## Technical Context

**Language/Version**: Python 3.11, Node.js 18+ (for Docusaurus), JavaScript/TypeScript for frontend components
**Primary Dependencies**: Docusaurus, FastAPI, Neon Postgres, Qdrant, OpenAI Agents/ChatKit SDKs, ROS 2 (Humble Hawksbill), NVIDIA Isaac Sim
**Storage**: PostgreSQL (Neon) for metadata, Qdrant (vector store) for embeddings, Git for content versioning
**Testing**: pytest for backend, Jest for frontend components, Docusaurus built-in testing, integration tests for RAG accuracy
**Target Platform**: Web-based (GitHub Pages for frontend, cloud hosting for backend API), compatible with ROS 2 and NVIDIA Isaac ecosystems
**Project Type**: Web application with separate frontend (Docusaurus) and backend (FastAPI)
**Performance Goals**: <200ms response time for RAG queries, <3s page load time, support 1000+ concurrent users
**Constraints**: Must use free-tier resources where possible (Qdrant/Neon), embedded chatbot in Docusaurus pages, selected-text context for queries
**Scale/Scope**: Educational content for CS students and developers, 6 chapters + capstone, ~10k words of content, vector database for RAG system

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution for Physical AI & Humanoid Robotics:
- ✅ Spec-Driven Development: Following Spec-Kit Plus workflow with proper specifications before implementation
- ✅ AI-First Authorship: Content generation will leverage Claude Code with human oversight
- ✅ Interactive Pedagogy: Implementation includes RAG chatbot for active querying alongside reading
- ✅ Embodied Intelligence: Focus on bridging digital AI with physical robotics concepts
- ✅ Modular Architecture: Decoupling book content from chatbot backend while maintaining integration

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-book-rag/
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
├── src/
│   ├── models/
│   │   ├── document.py
│   │   ├── embedding.py
│   │   └── query.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── document_service.py
│   │   └── embedding_service.py
│   ├── api/
│   │   ├── v1/
│   │   │   ├── documents.py
│   │   │   ├── embeddings.py
│   │   │   └── queries.py
│   │   └── main.py
│   └── config/
│       ├── database.py
│       └── settings.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/
├── docs/
│   ├── chapter-1-ros2.md
│   ├── chapter-2-simulation.md
│   ├── chapter-3-isaac.md
│   ├── chapter-4-vla.md
│   ├── chapter-5-capstone.md
│   └── chapter-6-rag-implementation.md
├── src/
│   ├── components/
│   │   ├── ChatBot/
│   │   │   ├── ChatBot.jsx
│   │   │   ├── ChatInterface.jsx
│   │   │   └── MessageRenderer.jsx
│   │   └── TextSelector/
│   │       └── TextSelector.jsx
│   └── pages/
├── docusaurus.config.js
├── sidebars.js
└── package.json
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (Docusaurus) to maintain modularity while allowing tight integration between the static documentation site and the RAG backend. The Docusaurus site will be deployed to GitHub Pages while the FastAPI backend will be hosted separately with Neon Postgres and Qdrant for the RAG system.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [No violations identified] | [All constitution requirements met] |