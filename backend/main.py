from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
import database
from models.request import ChatRequest
from models.response import ChatResponse
from services.chat_service import ChatService
from services.history_service import HistoryService
import os
from dotenv import load_dotenv

# Import the get_db dependency specifically
get_db = database.get_db

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(title="BiblioChat API", description="API for the BiblioChat RAG-based book reader application")

# Add CORS middleware to allow requests between SvelteKit localhost and FastAPI localhost
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
chat_service = ChatService()
history_service = HistoryService()

# API routes
@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest, db: Session = Depends(get_db)):
    """
    Process user query with book context using dual-mode logic:
    - Selection Mode: If selection_context is provided, bypasses vector search and sends text directly to AI
    - Global Mode: If selection_context is None, performs vector search in Qdrant and sends results to AI
    """
    try:
        # Determine mode based on selection_context
        mode = "selection" if chat_request.selection_context else "global"

        # Process the chat request using the chat service
        response = await chat_service.process_request(chat_request)

        # Save the interaction to history
        await history_service.save_interaction(
            db=db,
            user_message=chat_request.query,
            ai_response=response.response,
            mode=mode,
            session_id=chat_request.session_id
        )

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.get("/api/chat/history/{session_id}")
async def get_chat_history(session_id: str, db: Session = Depends(get_db)):
    """
    Retrieve chat history for a specific session
    """
    try:
        # Get the history service instance
        history_service = HistoryService()

        # Retrieve the session history
        history = await history_service.get_session_history(db, session_id)

        # Convert to a format suitable for response
        history_list = []
        for item in history:
            history_list.append({
                "id": item.id,
                "user_message": item.user_message,
                "ai_response": item.ai_response,
                "mode": item.mode,
                "timestamp": item.timestamp.isoformat() if item.timestamp else None
            })

        return {"session_id": session_id, "history": history_list}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving chat history: {str(e)}")

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "timestamp": "2025-12-18T10:30:00Z"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)