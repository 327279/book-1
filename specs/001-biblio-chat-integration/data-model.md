# Data Model: BiblioChat Backend

**Date**: 2025-12-18
**Feature**: BiblioChat Backend
**Branch**: 001-biblio-chat-integration

## Overview

This document defines the data models for the BiblioChat backend, including database schemas, API request/response models, and internal data structures.

## Database Models

### ChatHistory Model

**Purpose**: Stores user chat interactions with the book for persistence across sessions

**Fields**:
- `id` (Integer, Primary Key, Auto-increment)
  - Unique identifier for each chat entry
  - Required: Yes
  - Validation: Auto-generated, positive integer

- `user_message` (Text)
  - The user's query or message
  - Required: Yes
  - Validation: Not empty, max length appropriate for text

- `ai_response` (Text)
  - The AI-generated response to the user's query
  - Required: Yes
  - Validation: Not empty, max length appropriate for text

- `mode` (String, Enum: 'selection' | 'global')
  - Indicates whether the query was in selection mode or global mode
  - Required: Yes
  - Validation: Must be either 'selection' or 'global'

- `session_id` (String)
  - Identifier for grouping related chat messages into sessions
  - Required: Yes
  - Validation: UUID format or similar unique identifier

- `timestamp` (DateTime)
  - When the chat interaction occurred
  - Required: Yes
  - Validation: Current timestamp on creation

- `book_id` (String)
  - Identifier for the book being queried (for future multi-book support)
  - Required: Yes (default to single book ID)
  - Validation: String identifier

### SQLAlchemy Implementation

```python
from sqlalchemy import Column, Integer, String, Text, DateTime
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime

Base = declarative_base()

class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, index=True)
    user_message = Column(Text, nullable=False)
    ai_response = Column(Text, nullable=False)
    mode = Column(String(20), nullable=False)  # 'selection' or 'global'
    session_id = Column(String(255), nullable=False)
    book_id = Column(String(255), nullable=False, default="main_book")
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow)
```

## API Request/Response Models

### ChatRequest Model

**Purpose**: Defines the structure for incoming chat requests

**Fields**:
- `query` (String)
  - The user's question or query
  - Required: Yes
  - Type: String
  - Validation: Not empty, max length appropriate for AI context

- `selection_context` (String, Optional)
  - The text selected by the user in the book (if any)
  - Required: No (can be null)
  - Type: String | null
  - Validation: If provided, not empty

- `session_id` (String)
  - Identifier for the chat session
  - Required: Yes
  - Type: String
  - Validation: Should be UUID format or similar unique identifier

**Pydantic Implementation**:
```python
from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    query: str
    selection_context: Optional[str] = None
    session_id: str
```

### ChatResponse Model

**Purpose**: Defines the structure for outgoing chat responses

**Fields**:
- `response` (String)
  - The AI-generated response to the user's query
  - Required: Yes
  - Type: String
  - Validation: Not empty

- `sources` (List[String])
  - List of source references used in the AI response (for global mode)
  - Required: Yes
  - Type: List[str]
  - Validation: Array of strings, can be empty

**Pydantic Implementation**:
```python
from pydantic import BaseModel
from typing import List

class ChatResponse(BaseModel):
    response: str
    sources: List[str]
```

## Internal Data Structures

### VectorRecord Model

**Purpose**: Internal representation of a text chunk with its embedding for Qdrant storage

**Fields**:
- `id` (String)
  - Unique identifier for the vector record
  - Required: Yes
  - Type: String (UUID)

- `text_content` (String)
  - The actual text content of the chunk
  - Required: Yes
  - Type: String

- `embedding` (List[Float])
  - The 1024-dimensional vector embedding of the text
  - Required: Yes
  - Type: List[float]
  - Validation: Must be exactly 1024 elements

- `metadata` (Dict)
  - Additional metadata about the text chunk
  - Required: No
  - Type: Dict[str, Any]
  - Content: Book section, page number, etc.

### SearchResult Model

**Purpose**: Represents the result of a Qdrant vector search

**Fields**:
- `text_content` (String)
  - The text content of the matching chunk
  - Required: Yes
  - Type: String

- `similarity_score` (Float)
  - The similarity score from the vector search
  - Required: Yes
  - Type: Float
  - Validation: Between 0 and 1

- `metadata` (Dict)
  - Metadata associated with the matching chunk
  - Required: No
  - Type: Dict[str, Any]

## Relationships

### ChatHistory Relationships
- Each ChatHistory entry belongs to a specific `session_id`
- Each ChatHistory entry is associated with a `book_id` (for future multi-book support)
- Timestamp allows chronological ordering of chat messages within a session

## Validation Rules

### Database Constraints
- ChatHistory.user_message: NOT NULL, length > 0
- ChatHistory.ai_response: NOT NULL, length > 0
- ChatHistory.mode: CHECK (mode IN ('selection', 'global'))
- ChatHistory.session_id: NOT NULL, length > 0
- ChatHistory.timestamp: NOT NULL, defaults to current time

### API Validation
- ChatRequest.query: Required, non-empty string
- ChatRequest.session_id: Required, should be valid identifier format
- ChatRequest.selection_context: If provided, must be non-empty string
- ChatResponse.response: Required, non-empty string
- ChatResponse.sources: Required, must be an array (can be empty)

## Future Considerations

### Multi-book Support
- The `book_id` field in ChatHistory is already included to support future multi-book functionality
- Vector records could include book identifier for multi-book vector storage

### Enhanced Metadata
- Future versions could include page numbers, chapter information, or other book-specific metadata
- User preference data could be stored separately to customize AI behavior