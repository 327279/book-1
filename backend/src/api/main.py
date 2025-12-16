from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .v1 import documents, embeddings, queries, agents, auth
from ..config.settings import settings

# Create FastAPI app instance
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="API for the RAG (Retrieval Augmented Generation) system that powers the interactive chatbot in the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "*"],  # In production, specify the exact frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(documents.router)
app.include_router(embeddings.router)
app.include_router(queries.router)
app.include_router(agents.router)
app.include_router(auth.router)

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics RAG API", "version": "1.0.0"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "message": "RAG API is running", "version": "1.0.0"}

# For running with uvicorn
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.api.main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug
    )