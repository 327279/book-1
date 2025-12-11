---
title: "Chapter 6 - RAG Chatbot Implementation"
sidebar_position: 6
---

# RAG Chatbot Implementation

## Introduction to RAG for Robotics Education

Retrieval-Augmented Generation (RAG) systems combine the knowledge retrieval capabilities of information retrieval with the generative abilities of large language models. In the context of robotics education, RAG systems provide an interactive way for students to ask questions about complex robotics concepts and receive accurate, contextually relevant answers based on the textbook content.

This chapter covers the implementation of the "Book Bot" - an AI-powered assistant that can answer questions specifically about the robotics content in this textbook.

## Indexing Book Content into Qdrant

### Content Preprocessing

Before content can be indexed, it needs to be preprocessed into appropriate chunks:

```python
import re
from typing import List, Dict
from dataclasses import dataclass

@dataclass
class ContentChunk:
    id: str
    text: str
    metadata: Dict
    chapter: int
    section: str

class ContentPreprocessor:
    def __init__(self):
        self.chunk_size = 1000  # characters
        self.overlap = 100     # characters

    def preprocess_document(self, document_id: str, title: str, content: str, chapter_number: int):
        """Preprocess document content for RAG indexing"""
        # Split content into semantic chunks
        chunks = self.chunk_content(content, title, chapter_number)

        # Add metadata to each chunk
        processed_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_obj = ContentChunk(
                id=f"{document_id}_chunk_{i}",
                text=chunk["text"],
                metadata={
                    "chapter_number": chapter_number,
                    "section_title": chunk["section"],
                    "chunk_index": i,
                    "document_id": document_id
                },
                chapter=chapter_number,
                section=chunk["section"]
            )
            processed_chunks.append(chunk_obj)

        return processed_chunks

    def chunk_content(self, content: str, title: str, chapter_number: int):
        """Split content into semantic chunks"""
        chunks = []
        paragraphs = content.split('\n\n')

        current_chunk = ""
        current_section = title

        for para in paragraphs:
            # Check if this paragraph is a new section
            section_match = re.match(r'^#+\s+(.*)', para)
            if section_match:
                current_section = section_match.group(1).strip()

            # If adding this paragraph would exceed chunk size
            if len(current_chunk) + len(para) > self.chunk_size and current_chunk:
                # Add the current chunk
                chunks.append({
                    "text": current_chunk.strip(),
                    "section": current_section
                })

                # Start a new chunk with overlap
                overlap_start = max(0, len(current_chunk) - self.overlap)
                current_chunk = current_chunk[overlap_start:] + para
            else:
                current_chunk += para + "\n\n"

        # Add the last chunk
        if current_chunk.strip():
            chunks.append({
                "text": current_chunk.strip(),
                "section": current_section
            })

        return chunks
```

### Vector Embedding Generation

Generating embeddings for content chunks:

```python
import openai
from typing import List
import numpy as np

class EmbeddingGenerator:
    def __init__(self, api_key: str, model: str = "text-embedding-ada-002"):
        openai.api_key = api_key
        self.model = model

    def generate_embeddings(self, texts: List[str]):
        """Generate embeddings for a list of texts"""
        response = openai.Embedding.create(
            input=texts,
            model=self.model
        )

        embeddings = []
        for item in response['data']:
            embeddings.append(item['embedding'])

        return embeddings

    def generate_single_embedding(self, text: str):
        """Generate embedding for a single text"""
        return self.generate_embeddings([text])[0]
```

### Qdrant Integration

Storing embeddings in Qdrant vector database:

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid
from typing import List, Dict
import numpy as np

class QdrantIndexer:
    def __init__(self, url: str, api_key: str, collection_name: str = "robotics_book"):
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """Create Qdrant collection if it doesn't exist"""
        try:
            self.client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)
            )

    def index_chunks(self, chunks: List[ContentChunk], embedding_generator: EmbeddingGenerator):
        """Index content chunks into Qdrant"""
        # Prepare data for batch processing
        texts = [chunk.text for chunk in chunks]

        # Generate embeddings
        embeddings = embedding_generator.generate_embeddings(texts)

        # Prepare points for insertion
        points = []
        for chunk, embedding in zip(chunks, embeddings):
            points.append(
                models.PointStruct(
                    id=chunk.id,
                    vector=embedding,
                    payload={
                        "text": chunk.text,
                        "chapter": chunk.chapter,
                        "section": chunk.section,
                        "metadata": chunk.metadata
                    }
                )
            )

        # Upload to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return len(points)
```

## Building the FastAPI Backend with Neon Postgres

### Database Models and Services

```python
from sqlalchemy import create_engine, Column, String, Text, DateTime, Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.sql import func
import uuid

Base = declarative_base()

class UserQuery(Base):
    __tablename__ = "user_queries"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, index=True)
    query_text = Column(Text)
    selected_text = Column(Text)
    response_text = Column(Text)
    source_chunks = Column(Text)  # JSON string of source chunks
    confidence_score = Column(Float)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

# Database service for managing queries
class QueryService:
    @staticmethod
    def save_query(db_session, user_id: str, query_text: str, selected_text: str,
                   response_text: str, source_chunks: list, confidence_score: float):
        """Save user query and response to database"""
        query = UserQuery(
            user_id=user_id,
            query_text=query_text,
            selected_text=selected_text,
            response_text=response_response,
            source_chunks=str(source_chunks),  # In production, use JSON serialization
            confidence_score=confidence_score
        )
        db_session.add(query)
        db_session.commit()
        return query.id
```

### RAG Service Implementation

```python
from typing import List, Dict, Tuple
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self, qdrant_indexer: QdrantIndexer, embedding_generator: EmbeddingGenerator):
        self.qdrant = qdrant_indexer
        self.embedding_gen = embedding_generator

    def query(self, user_query: str, selected_text: str = None, top_k: int = 5) -> Tuple[str, List[Dict], float]:
        """Process user query using RAG"""
        try:
            # Generate embedding for the query
            query_embedding = self.embedding_gen.generate_single_embedding(user_query)

            # Search in vector database
            search_results = self.qdrant.client.search(
                collection_name=self.qdrant.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Extract relevant chunks
            relevant_chunks = []
            for result in search_results:
                relevant_chunks.append({
                    "text": result.payload["text"],
                    "chapter": result.payload["chapter"],
                    "section": result.payload["section"],
                    "score": result.score
                })

            # Generate response using context from relevant chunks
            response = self._generate_response(user_query, relevant_chunks, selected_text)

            # Calculate confidence based on similarity scores
            avg_score = sum(r["score"] for r in relevant_chunks) / len(relevant_chunks) if relevant_chunks else 0
            confidence = min(avg_score * 2, 1.0)  # Normalize to 0-1 range

            return response, relevant_chunks, confidence

        except Exception as e:
            logger.error(f"Error in RAG query: {str(e)}")
            return f"Sorry, I encountered an error processing your query: {str(e)}", [], 0.0

    def _generate_response(self, query: str, context_chunks: List[Dict], selected_text: str = None):
        """Generate response based on query and context"""
        # Build context from relevant chunks
        context = "\n\n".join([chunk["text"] for chunk in context_chunks[:3]])  # Use top 3 chunks

        # If user selected specific text, prioritize that in the response
        if selected_text:
            prompt = f"""
            Based on the following text from the robotics textbook:

            {context}

            The user has specifically selected this text: "{selected_text}"

            Please answer the user's question: "{query}"

            Provide a detailed answer based on the textbook content, referencing specific concepts mentioned in the provided text.
            """
        else:
            prompt = f"""
            Based on the following text from the robotics textbook:

            {context}

            Please answer the user's question: "{query}"

            Provide a detailed answer based on the textbook content, referencing specific concepts mentioned in the provided text.
            """

        # In a real implementation, this would call an LLM API
        # For this example, we'll simulate the response
        response = f"Based on the textbook content, here's information about '{query}'. The relevant sections discuss important concepts related to robotics and AI. Please refer to the specific chapters for detailed explanations."

        return response
```

### FastAPI Endpoints

```python
from fastapi import FastAPI, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional, List
import uuid

app = FastAPI(title="Robotics Book RAG API")

class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    user_id: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    sources: List[Dict]
    confidence: float
    query_id: str

@app.post("/api/v1/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest, db: Session = Depends(get_db)):
    """Endpoint for querying the RAG system"""
    if not request.query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    # Process the query using RAG service
    response_text, sources, confidence = rag_service.query(
        request.query,
        request.selected_text
    )

    # Save query to database
    query_id = QueryService.save_query(
        db,
        request.user_id or "anonymous",
        request.query,
        request.selected_text,
        response_text,
        sources,
        confidence
    )

    return QueryResponse(
        response=response_text,
        sources=sources,
        confidence=confidence,
        query_id=query_id
    )

@app.get("/api/v1/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "RAG API"}
```

## Integrating the Chat UI into Docusaurus

### Creating the Chat Component

```jsx
// frontend/src/components/ChatBot/ChatBot.jsx
import React, { useState, useRef, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import './ChatBot.css';

const ChatBot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const { colorMode } = useColorMode();

  // Function to get currently selected text
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare request with selected text context
      const response = await fetch('/api/v1/query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: inputValue,
          selected_text: selectedText,
          user_id: 'docusaurus-user'
        })
      });

      const data = await response.json();

      // Add bot response
      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources,
        confidence: data.confidence
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request.',
        sender: 'bot',
        error: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after use
    }
  };

  return (
    <div className={`chat-container ${colorMode}`}>
      <div className="chat-header">
        <h3>Book Bot - Robotics Assistant</h3>
        {selectedText && (
          <div className="selected-text-preview">
            <small>Context: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</small>
          </div>
        )}
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your Robotics Book Assistant.</p>
            <p>Ask me questions about the content, or select text on the page and ask about it specifically!</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender} ${message.error ? 'error' : ''}`}
            >
              <div className="message-content">
                <p>{message.text}</p>
                {message.sources && message.sources.length > 0 && (
                  <div className="sources">
                    <small>Sources: {message.sources.map(s => `Ch. ${s.chapter}`).join(', ')}</small>
                  </div>
                )}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot">
            <div className="message-content">
              <p>Thinking...</p>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form className="chat-input-form" onSubmit={handleSubmit}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about robotics..."
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading || !inputValue.trim()}>
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default ChatBot;
```

### CSS Styling for the Chat Component

```css
/* frontend/src/components/ChatBot/ChatBot.css */
.chat-container {
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  height: 500px;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.chat-container.light {
  background-color: white;
}

.chat-container.dark {
  background-color: #1c1e21;
}

.chat-header {
  padding: 1rem;
  border-bottom: 1px solid var(--ifm-color-emphasis-300);
  background-color: var(--ifm-color-primary-dark);
  color: white;
}

.chat-header h3 {
  margin: 0;
  font-size: 1.1rem;
}

.selected-text-preview {
  margin-top: 0.5rem;
  font-size: 0.8rem;
  opacity: 0.8;
}

.chat-messages {
  flex: 1;
  overflow-y: auto;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.welcome-message {
  text-align: center;
  color: var(--ifm-color-emphasis-600);
  font-style: italic;
}

.message {
  max-width: 80%;
  padding: 0.75rem;
  border-radius: 8px;
  word-wrap: break-word;
}

.message.user {
  align-self: flex-end;
  background-color: var(--ifm-color-primary);
  color: white;
}

.message.bot {
  align-self: flex-start;
  background-color: var(--ifm-color-emphasis-200);
}

.message.error {
  background-color: #fee;
  color: #c33;
}

.sources {
  margin-top: 0.5rem;
  font-size: 0.8rem;
  opacity: 0.7;
}

.chat-input-form {
  display: flex;
  padding: 1rem;
  border-top: 1px solid var(--ifm-color-emphasis-300);
  background-color: inherit;
}

.chat-input-form input {
  flex: 1;
  padding: 0.5rem;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 4px;
  margin-right: 0.5rem;
}

.chat-input-form button {
  padding: 0.5rem 1rem;
  background-color: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.chat-input-form button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}
```

### Integrating Chat Component into Docusaurus Layout

To integrate the chat component into the Docusaurus layout, you can add it as a sidebar element or as a floating button. Here's how to add it as a component in the page:

```jsx
// Example of how to use the ChatBot component in a Docusaurus page
import ChatBot from '@site/src/components/ChatBot/ChatBot';

function MyPage() {
  return (
    <div>
      <main>
        {/* Your page content */}
        <h1>Robotics Content</h1>
        <p>This is the main content of the page...</p>
      </main>

      {/* Floating chat button that expands to full chat interface */}
      <div className="chat-float">
        <ChatBot />
      </div>
    </div>
  );
}
```

## Testing the RAG Implementation

### Unit Tests for RAG Components

```python
import unittest
from unittest.mock import Mock, patch
import numpy as np

class TestRAGService(unittest.TestCase):
    def setUp(self):
        self.mock_qdrant = Mock()
        self.mock_embedding_gen = Mock()
        self.rag_service = RAGService(self.mock_qdrant, self.mock_embedding_gen)

    def test_query_with_context(self):
        """Test that queries work with selected text context"""
        # Mock the embedding generation
        self.mock_embedding_gen.generate_single_embedding.return_value = [0.1, 0.2, 0.3]

        # Mock the Qdrant search results
        mock_result = Mock()
        mock_result.payload = {"text": "Sample context", "chapter": 1, "section": "Intro"}
        mock_result.score = 0.8
        self.mock_qdrant.client.search.return_value = [mock_result]

        # Test the query
        response, sources, confidence = self.rag_service.query(
            "What is ROS?",
            selected_text="This text talks about ROS"
        )

        # Assertions
        self.mock_embedding_gen.generate_single_embedding.assert_called_once()
        self.mock_qdrant.client.search.assert_called_once()
        self.assertIsInstance(response, str)
        self.assertEqual(len(sources), 1)
        self.assertGreater(confidence, 0)

    @patch('requests.post')
    def test_api_endpoint(self, mock_post):
        """Test the FastAPI endpoint"""
        # This would test the actual API endpoint
        # Implementation would use TestClient from FastAPI
        pass

if __name__ == '__main__':
    unittest.main()
```

## Performance Optimization

### Caching Strategies

```python
from functools import lru_cache
import hashlib

class CachedRAGService(RAGService):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cache = {}

    @lru_cache(maxsize=128)
    def cached_query(self, query_hash: str, query: str, selected_text: str = None):
        """Cached version of query method"""
        return self.query(query, selected_text)

    def query(self, user_query: str, selected_text: str = None, top_k: int = 5):
        """Query with caching"""
        # Create a hash of the query and selected text for caching
        cache_key = hashlib.md5(f"{user_query}_{selected_text}".encode()).hexdigest()

        # Check if result is in cache
        if cache_key in self.cache:
            return self.cache[cache_key]

        # Perform the actual query
        result = super().query(user_query, selected_text, top_k)

        # Cache the result (with size limits in production)
        self.cache[cache_key] = result

        return result
```

## Summary

This chapter covered the complete implementation of the RAG chatbot system:

- Content indexing using semantic chunking and vector embeddings
- Qdrant vector database integration for efficient similarity search
- FastAPI backend with PostgreSQL logging
- Docusaurus frontend integration with text selection capabilities
- Testing and performance optimization strategies

The RAG system enables interactive learning by allowing students to ask questions about specific parts of the textbook content and receive AI-generated responses grounded in the educational material. This creates a personalized learning experience that adapts to each student's specific questions and areas of interest.