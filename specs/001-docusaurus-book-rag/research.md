# Research Summary: Physical AI & Humanoid Robotics: Docusaurus Book & RAG Chatbot

## RAG Text Chunking Strategy

**Decision**: Use semantic chunking with sentence-level boundaries for optimal context preservation
**Rationale**: For educational content about robotics, preserving context around code examples, technical explanations, and related concepts is crucial. Semantic chunking ensures that related concepts stay together while maintaining reasonable vector size for the RAG system.
**Alternatives considered**:
- Fixed-size character chunks: Risked breaking up related technical concepts
- Sentence-level chunks: Could create too many small chunks, reducing context
- Document-level chunks: Would exceed vector database limits and reduce precision

## ChatKit React Component Embedding Approach

**Decision**: Create a custom React component that integrates with Docusaurus's MDX system
**Rationale**: This allows for tight integration with the documentation site while maintaining the ability to select text and provide context to the RAG system. The component will be able to capture selected text and send it as context to the backend API.
**Alternatives considered**:
- Using an iframe: Would create isolation issues and complicate text selection
- Full custom implementation: Would require more development time than necessary
- Third-party chat widgets: Might not provide the specific text selection functionality needed

## ROS 2/Isaac Version Compatibility Choices

**Decision**: Target ROS 2 Humble Hawksbill (LTS) and NVIDIA Isaac Sim 2023.1.0
**Rationale**: Humble Hawksbill is the latest LTS version providing long-term support and stability for educational content. Isaac Sim 2023.1.0 provides the necessary features for the educational objectives while being stable enough for students to use.
**Alternatives considered**:
- Rolling Ridley: Less stable for educational use
- Iron Irwini: Newer but not LTS, potentially less stable
- Earlier versions: Would miss newer features important for learning

## Docusaurus Integration Strategy

**Decision**: Use Docusaurus MDX components for embedding the RAG chatbot
**Rationale**: MDX allows React components to be embedded directly in Markdown files, providing a seamless experience for users to interact with the chatbot while reading content.
**Alternatives considered**:
- Standard React components: Would require more complex setup
- Separate chat page: Would break the flow of learning by separating content from interaction

## GitHub Pages Deployment Strategy

**Decision**: Use Docusaurus's built-in GitHub Pages deployment with a custom domain
**Rationale**: Docusaurus has excellent built-in support for GitHub Pages, making deployment straightforward and cost-effective as required by the constraints.
**Alternatives considered**:
- Netlify/Vercel: Would add unnecessary complexity and potential costs
- Self-hosted: Would violate the constraint of using free-tier resources

## Vector Database Strategy (Qdrant)

**Decision**: Use Qdrant Cloud Free Tier with semantic embeddings using Sentence Transformers
**Rationale**: Qdrant provides excellent performance for semantic search and is specifically designed for vector similarity search. The free tier meets the cost constraints while providing sufficient capabilities.
**Alternatives considered**:
- Pinecone: Would likely exceed free tier constraints
- ChromaDB: Self-hosted option but would require additional infrastructure
- Weaviate: Alternative vector database but Qdrant has better free tier for this use case

## Backend API Design (FastAPI)

**Decision**: Implement RESTful API with OpenAPI documentation for the RAG system
**Rationale**: FastAPI provides automatic API documentation, type validation, and excellent performance. It integrates well with the Python ecosystem used for ROS 2 and robotics development.
**Alternatives considered**:
- GraphQL: Would add complexity without significant benefit for this use case
- Flask: Less feature-rich than FastAPI
- Node.js: Would create inconsistency with the Python/ROS 2 ecosystem