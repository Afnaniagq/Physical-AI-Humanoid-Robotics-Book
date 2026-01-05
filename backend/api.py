from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(title="AI Agent Chat API", version="1.0.0")

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Allow Docusaurus frontend
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods (GET, POST, PUT, DELETE, etc.)
    allow_headers=["*"],  # Allow all headers
    # Additional origins can be added as needed
    # allow_origins=["http://localhost:3000", "http://127.0.0.1:3000", "https://yourdomain.com"]
)

@app.get("/")
def read_root():
    return {"message": "AI Agent Chat API is running!"}


# Import models and utilities after they are defined
from backend.models.chat import ChatRequest, ChatResponse
from backend.models.error import APIError
from backend.utils.errors import APIErrorHandler, handle_agent_error
from backend.agent import ConversationManager
import asyncio
from typing import Dict

# In-memory storage for conversation managers (in production, use Redis or database)
conversation_storage: Dict[str, ConversationManager] = {}


def get_conversation_manager(session_id: str) -> ConversationManager:
    """
    Get or create a conversation manager for the given session ID
    """
    if session_id not in conversation_storage:
        conversation_storage[session_id] = ConversationManager()
    return conversation_storage[session_id]


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest):
    """
    Endpoint to send user queries to the AI agent and receive formatted responses with preserved Markdown.
    """
    from backend.utils.logging import chat_logger
    import time

    start_time = time.time()

    try:
        # Log the incoming request
        chat_logger.log_chat_request(
            session_id=chat_request.session_id,
            message=chat_request.message,
            ip_address=None  # In a real implementation, get from request.client
        )

        # Additional input validation and sanitization
        # Limit message length to prevent abuse
        if len(chat_request.message) > 10000:
            raise HTTPException(
                status_code=400,
                detail="Message too long. Maximum 10000 characters allowed."
            )

        # Validate session ID format to prevent injection
        import re
        if not re.match(r'^[a-zA-Z0-9_-]+$', chat_request.session_id):
            raise HTTPException(
                status_code=400,
                detail="Invalid session ID format. Only alphanumeric characters, hyphens, and underscores are allowed."
            )

        # Get or create conversation manager for the session
        conversation_manager = get_conversation_manager(chat_request.session_id)

        # Get the response from the agent
        agent_response = await conversation_manager.chat(chat_request.message)

        # Extract source URLs if they exist in the response
        # This is a simple implementation - in practice, you'd want to extract URLs more robustly
        response_str = str(agent_response)
        source_urls = []

        # Simple URL extraction (in a real implementation, use regex or a proper parser)
        url_pattern = r'https?://[^\s<>"{}|\\^`\[\]]+'
        urls = re.findall(url_pattern, response_str)
        if urls:
            source_urls = list(set(urls))  # Remove duplicates

        # Calculate processing time
        processing_time = time.time() - start_time

        # Log the successful response
        chat_logger.log_chat_response(
            session_id=chat_request.session_id,
            response=response_str,
            processing_time=processing_time
        )

        # Create and return the response
        return ChatResponse(
            response=response_str,
            session_id=chat_request.session_id,
            source_urls=source_urls
        )

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Log the error
        from backend.utils.errors import handle_agent_error
        chat_logger.log_error(chat_request.session_id, str(e), error_type=type(e).__name__)

        # Handle any errors that occur during the chat process
        error_response = handle_agent_error(e)
        raise HTTPException(
            status_code=error_response.status_code,
            detail=error_response.error
        )


# Import and include routes here after they are defined
# from .routers import chat
# app.include_router(chat.router, prefix="/api/v1", tags=["chat"])