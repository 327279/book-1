# Data Model: Physical AI & Humanoid Robotics: Docusaurus Book & RAG Chatbot

## Document Entity

**Description**: Represents a chapter or section of the educational content
- **Fields**:
  - `id`: Unique identifier for the document
  - `title`: Title of the chapter/section
  - `content`: Full text content of the document
  - `chapter_number`: Sequential number of the chapter (1-6)
  - `module`: Module identifier (e.g., "ROS2", "Simulation", "Isaac", "VLA", "Capstone", "RAG")
  - `created_at`: Timestamp of creation
  - `updated_at`: Timestamp of last update
  - `version`: Version number for content tracking
- **Relationships**: None
- **Validation**: Content must be non-empty, chapter_number must be between 1-6

## Embedding Entity

**Description**: Vector representation of document chunks for RAG system
- **Fields**:
  - `id`: Unique identifier for the embedding
  - `document_id`: Reference to the source document
  - `chunk_text`: Original text chunk that was embedded
  - `chunk_index`: Position of this chunk within the document
  - `vector`: High-dimensional vector representation (handled by Qdrant)
  - `metadata`: Additional metadata about the chunk
- **Relationships**: Belongs to Document
- **Validation**: Vector must be properly formatted, document_id must reference existing document

## Query Entity

**Description**: Represents a user query to the RAG system
- **Fields**:
  - `id`: Unique identifier for the query
  - `user_id`: Identifier for the user (optional for anonymous usage)
  - `query_text`: Original text of the user's query
  - `selected_text`: Text selected by user when making the query (optional)
  - `context`: Additional context provided with the query
  - `created_at`: Timestamp of the query
  - `session_id`: Identifier for the chat session
- **Relationships**: None
- **Validation**: Query_text must be non-empty

## QueryResult Entity

**Description**: Represents the response from the RAG system
- **Fields**:
  - `id`: Unique identifier for the result
  - `query_id`: Reference to the original query
  - `response_text`: AI-generated response to the query
  - `source_documents`: List of document IDs used to generate the response
  - `confidence_score`: Confidence level of the response (0-1)
  - `created_at`: Timestamp of response generation
  - `retrieved_chunks`: List of chunk IDs used to generate the response
- **Relationships**: Belongs to Query
- **Validation**: Response must be non-empty, confidence_score must be between 0-1

## UserSession Entity

**Description**: Represents a user's interaction session with the chatbot
- **Fields**:
  - `id`: Unique identifier for the session
  - `user_id`: Identifier for the user (optional)
  - `created_at`: Timestamp of session creation
  - `updated_at`: Timestamp of last activity
  - `page_url`: URL of the page where the session started
  - `active`: Boolean indicating if the session is currently active
- **Relationships**: Contains multiple Queries
- **Validation**: Must have either user_id or be associated with a page_url

## CodeExample Entity

**Description**: Represents executable code examples within the educational content
- **Fields**:
  - `id`: Unique identifier for the code example
  - `document_id`: Reference to the containing document
  - `title`: Brief description of the code example
  - `language`: Programming language (e.g., "python", "bash", "yaml")
  - `code`: The actual code content
  - `description`: Explanation of what the code does
  - `execution_context`: Environment requirements for running the code
- **Relationships**: Belongs to Document
- **Validation**: Code must be non-empty, language must be valid

## State Transitions

### Query Processing
1. User submits a query with optional selected text
2. Query entity is created with the input
3. RAG system retrieves relevant document chunks
4. AI generates response based on retrieved context
5. QueryResult entity is created with the response
6. Response is returned to the user

### Document Processing
1. Educational content is added as a Document entity
2. Content is chunked into appropriate segments
3. Each chunk is converted to an Embedding entity
4. Embeddings are stored in the vector database (Qdrant)
5. Document is available for RAG queries