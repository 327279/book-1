# Feature Specification: BiblioChat - AI-Native Integrated RAG Book Reader

**Feature Branch**: `001-biblio-chat-integration`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot Implementation (BiblioChat)
Target audience: Full-stack developers deploying a production-ready AI book reader.
Focus: Integration of SvelteKit (UI), FastAPI (Logic), and Cohere/Qdrant/Neon (Data & AI Layer).
Success criteria:
Functional Vector Pipeline: System ingests book text, embeds it using Cohere embed-english-v3.0 (1024 dims), and indexes it in Qdrant.
Dual-Mode Chat Logic:
Selection Mode: If user highlights text in SvelteKit, Backend bypasses search and sends text + prompt to Cohere.
Global Mode: If no selection, Backend searches Qdrant -> Reranks -> Sends context to Cohere.
Stateful History: User chat history is persisted in Neon (Postgres).
Latency: Text selection to answer generation is under 2 seconds.
Constraints (Hardcoded Credentials):
The following credentials must be used to configure the environment variables or application constants immediately:
Neon Database (Postgres):
Connection String: postgresql://neondb_owner:npg_FxR5tiXI3dkv@ep-muddy-frog-ah6jzfcs-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
Library: Use asyncpg with SQLAlchemy.
Qdrant (Vector DB):
Cluster ID: 6cfdd60d-3649-4547-acb0-7aa88ce8444d
URL Construction: Construct the endpoint as https://6cfdd60d-3649-4547-acb0-7aa88ce8444d.us-east4-0.gcp.cloud.qdrant.io (Standard GCP Cluster URL).
API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.info0CDXMgdhMcapDEZQkbAZFBXBWIEooEkipqT64co
Collection Name: biblio_chat
Vector Size: 1024 (Strict requirement for Cohere English v3).
Cohere (AI Engine):
API Key: 7ys16G9aChXiWXxmhWuxL9N42pFdEe8bSvKjVRl7
Embedding Model: embed-english-v3.0
Chat Model: command-r
Technical Specifications:
Frontend: SvelteKit. Use window.getSelection() to capture text.
Backend: FastAPI.
Middleware: CORS must allow requests between SvelteKit localhost and FastAPI localhost.
Not building:
User Authentication (Hardcode user_id=1 for now).
PDF Parsing (Assume input is book.txt or a hardcoded string).
Multi-book support (Focus on single book architecture)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book Text Interaction (Priority: P1)

As a reader, I want to select text in a book and ask questions about it so that I can get immediate, context-aware explanations and insights.

**Why this priority**: This is the core value proposition of the product - allowing users to interact with book content through AI-powered explanations, which is the primary differentiator of the application.

**Independent Test**: Can be fully tested by selecting text in a book and submitting a question, which should return an AI-generated response based on the selected text context, delivering immediate value to the user.

**Acceptance Scenarios**:

1. **Given** user is viewing book content, **When** user selects text and asks a question, **Then** the system returns an AI response based on the selected text context within 2 seconds
2. **Given** user has selected text in the book, **When** user submits a question about the selected text, **Then** the system bypasses global search and sends the selected text directly to the AI engine

---

### User Story 2 - Global Book Search (Priority: P2)

As a reader, I want to ask questions about the entire book without selecting specific text so that I can get comprehensive answers based on the full book content.

**Why this priority**: This provides the secondary core functionality that allows users to search the entire book for information without needing to select specific text, expanding the utility of the system.

**Independent Test**: Can be fully tested by submitting a question without text selection, which should trigger a vector search across the book content and return relevant AI-generated responses.

**Acceptance Scenarios**:

1. **Given** user is viewing book content, **When** user submits a question without text selection, **Then** the system performs a vector search in Qdrant and returns AI-generated response based on relevant book sections
2. **Given** user has submitted a global query, **When** the system retrieves relevant book sections, **Then** the AI response is generated using the retrieved context with proper citations

---

### User Story 3 - Conversation History Persistence (Priority: P3)

As a reader, I want my chat history with the book to be preserved so that I can continue conversations across sessions and reference previous interactions.

**Why this priority**: This enhances user experience by providing continuity and allowing users to build upon previous conversations with the book content.

**Independent Test**: Can be fully tested by having a conversation with the book, closing the application, and then reopening to see that the conversation history is preserved.

**Acceptance Scenarios**:

1. **Given** user has had a conversation with the book, **When** user returns to the application, **Then** the previous conversation history is displayed
2. **Given** user has multiple chat sessions, **When** user navigates between sessions, **Then** each session's history is preserved independently

---

### Edge Cases

- What happens when the AI response generation takes longer than 2 seconds?
- How does the system handle very long text selections that might exceed AI context limits?
- What happens when the vector database is temporarily unavailable?
- How does the system handle network failures during AI processing?
- What happens when a user tries to select text that spans multiple book sections?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to select text in the book interface using window.getSelection() functionality
- **FR-002**: System MUST send selected text directly to Cohere AI engine when text is highlighted, bypassing vector search
- **FR-003**: System MUST perform vector search in Qdrant when no text is selected and send results to Cohere
- **FR-004**: System MUST store and retrieve user chat history in Neon Postgres database
- **FR-005**: System MUST implement dual-mode chat logic (selection mode vs global mode) based on whether text is selected
- **FR-006**: System MUST ensure text selection to AI response generation completes within 2 seconds
- **FR-007**: System MUST support CORS between SvelteKit frontend and FastAPI backend on localhost
- **FR-008**: System MUST use Cohere embed-english-v3.0 model for embeddings with 1024 dimensions
- **FR-009**: System MUST index book content in Qdrant collection named 'biblio_chat' with 1024-dimensional vectors
- **FR-010**: System MUST use Cohere command-r model for chat responses
- **FR-011**: System MUST implement proper error handling when vector database is unavailable
- **FR-012**: System MUST maintain conversation context across multiple interactions

### Key Entities

- **ChatSession**: Represents a user's conversation with a book, including message history and metadata
- **BookContent**: Represents the text content of a book that has been processed and indexed in the vector database
- **VectorEmbedding**: Represents the 1024-dimensional vector representation of book text chunks stored in Qdrant
- **UserQuery**: Represents a user's input (either with or without selected text) that triggers AI processing

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Text selection to answer generation completes within 2 seconds for 95% of queries
- **SC-002**: Users can successfully interact with book content through both selection mode and global search mode
- **SC-003**: Chat history is persisted across sessions with 99% reliability
- **SC-004**: Vector pipeline successfully ingests book text, creates embeddings using Cohere, and indexes in Qdrant with 100% of content preserved
- **SC-005**: System maintains conversation context and provides contextually relevant responses to 90% of follow-up questions
- **SC-006**: Dual-mode chat functionality correctly switches between selection mode and global mode based on user interaction