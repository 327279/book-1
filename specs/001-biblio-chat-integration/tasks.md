---
description: "Task list for BiblioChat backend implementation"
---

# Tasks: BiblioChat Backend

**Input**: Design documents from `/specs/001-biblio-chat-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app backend**: `backend/` at repository root
- Paths shown below follow the structure defined in plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan in backend/
- [x] T002 Create requirements.txt with FastAPI, Cohere, Qdrant-client, SQLAlchemy, asyncpg, python-dotenv
- [x] T003 [P] Create .env file with Neon, Qdrant, and Cohere credentials
- [x] T004 Create .gitignore for Python project

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Setup SQLAlchemy database models in backend/database.py
- [x] T006 [P] Create ChatHistory model in backend/database.py following data-model.md specification
- [x] T007 [P] Setup database connection using asyncpg and Neon credentials in backend/database.py
- [x] T008 Setup FastAPI application in backend/main.py with CORS middleware
- [x] T009 [P] Create Pydantic models for API requests/responses in backend/models/
- [x] T010 [P] Create ChatRequest model in backend/models/request.py
- [x] T011 [P] Create ChatResponse model in backend/models/response.py
- [x] T012 Configure environment variables loading with python-dotenv

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book Text Interaction (Priority: P1) 🎯 MVP

**Goal**: Enable users to select text in a book and ask questions about it to get immediate, context-aware explanations and insights

**Independent Test**: Can be fully tested by selecting text in a book and submitting a question, which should return an AI-generated response based on the selected text context within 2 seconds

### Implementation for User Story 1

- [x] T013 [P] [US1] Create chat service in backend/services/chat_service.py
- [x] T014 [US1] Implement selection mode logic in backend/services/chat_service.py (bypass vector search when selection_context provided)
- [x] T015 [US1] Create embedding service in backend/services/embedding_service.py
- [x] T016 [US1] Implement history service in backend/services/history_service.py
- [x] T017 [US1] Create POST /api/chat endpoint in backend/main.py
- [x] T018 [US1] Add selection mode logic to chat endpoint (when selection_context is not null)
- [x] T019 [US1] Connect Cohere API for direct text processing in selection mode
- [x] T020 [US1] Add database persistence for chat history in selection mode
- [ ] T021 [US1] Validate response time under 2 seconds for selection mode

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Global Book Search (Priority: P2)

**Goal**: Enable users to ask questions about the entire book without selecting specific text to get comprehensive answers based on the full book content

**Independent Test**: Can be fully tested by submitting a question without text selection, which should trigger a vector search across the book content and return relevant AI-generated responses

### Implementation for User Story 2

- [x] T022 [P] [US2] Create vector setup script in backend/vector_setup.py
- [x] T023 [US2] Implement Qdrant connection and collection setup in backend/vector_setup.py
- [x] T024 [P] [US2] Create ingestion script in backend/ingest.py
- [x] T025 [US2] Implement text chunking logic in backend/ingest.py (1000 chars with 100 char overlap)
- [x] T026 [US2] Implement embedding generation using Cohere embed-english-v3.0 in backend/ingest.py
- [x] T027 [US2] Implement vector upsert to Qdrant in backend/ingest.py
- [x] T028 [US2] Add global mode logic to chat service in backend/services/chat_service.py (when selection_context is null)
- [x] T029 [US2] Implement vector search in Qdrant in backend/services/chat_service.py
- [x] T030 [US2] Add global mode logic to chat endpoint in backend/main.py
- [ ] T031 [US2] Validate response time under 2 seconds for global mode

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Conversation History Persistence (Priority: P3)

**Goal**: Preserve user chat history with the book so users can continue conversations across sessions and reference previous interactions

**Independent Test**: Can be fully tested by having a conversation with the book, closing the application, and then reopening to see that the conversation history is preserved

### Implementation for User Story 3

- [x] T032 [P] [US3] Enhance history service for session management in backend/services/history_service.py
- [x] T033 [US3] Add session creation and retrieval functionality in backend/services/history_service.py
- [x] T034 [US3] Add chat history retrieval by session in backend/services/history_service.py
- [x] T035 [US3] Update chat endpoint to save history in all modes
- [x] T036 [US3] Add GET endpoint to retrieve chat history for a session in backend/main.py
- [ ] T037 [US3] Validate chat history persistence across sessions
- [ ] T038 [US3] Test multi-session functionality

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T039 [P] Create health check endpoint GET /api/health in backend/main.py
- [ ] T040 [P] Add comprehensive error handling in all services
- [ ] T041 [P] Add request/response logging middleware
- [ ] T042 Add proper exception handling for external service failures (Cohere, Qdrant, Neon)
- [ ] T043 [P] Create init_db.py script for database initialization
- [ ] T044 [P] Add proper validation for input parameters
- [ ] T045 Add unit tests for critical components
- [ ] T046 Run quickstart.md validation to ensure complete functionality

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds upon foundational components but independent of US1
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May use history service but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all services for User Story 1 together:
Task: "Create chat service in backend/services/chat_service.py"
Task: "Create embedding service in backend/services/embedding_service.py"
Task: "Create history service in backend/services/history_service.py"
Task: "Create Pydantic models for API requests/responses in backend/models/"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [US1], [US2], [US3] labels map tasks to specific user stories for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence