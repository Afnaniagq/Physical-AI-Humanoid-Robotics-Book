from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class ChatRequest(BaseModel):
    """
    Represents a user's chat message and session information sent to the API
    """
    message: str = Field(
        ...,
        description="The user's query or message",
        min_length=1,
        max_length=10000
    )
    session_id: str = Field(
        ...,
        description="Unique identifier for the conversation session"
    )

    class Config:
        schema_extra = {
            "example": {
                "message": "What is embodied intelligence?",
                "session_id": "sess_abc123xyz"
            }
        }


class ChatResponse(BaseModel):
    """
    Represents the AI agent's response to a user's query
    """
    response: str = Field(
        ...,
        description="The AI agent's response with Markdown formatting preserved",
        max_length=50000
    )
    session_id: str = Field(
        ...,
        description="The session identifier from the request"
    )
    source_urls: Optional[List[str]] = Field(
        default_factory=list,
        description="List of source URLs referenced in the response"
    )
    timestamp: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat() + "Z",
        description="ISO 8601 timestamp of when the response was generated"
    )

    class Config:
        schema_extra = {
            "example": {
                "response": "**Embodied intelligence** refers to AI systems that interact with the physical world through sensors and actuators. This concept is crucial for humanoid robotics as it emphasizes the importance of environmental interaction in developing intelligent behavior.",
                "session_id": "sess_abc123xyz",
                "source_urls": [
                    "https://example.com/embodied-intelligence",
                    "https://example.com/humanoid-robotics"
                ],
                "timestamp": "2025-12-28T10:30:00Z"
            }
        }


class APIError(BaseModel):
    """
    Represents an error response from the API
    """
    error: str = Field(
        ...,
        description="Descriptive error message"
    )
    status_code: int = Field(
        ...,
        description="HTTP status code",
        ge=400,
        le=599
    )
    timestamp: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat() + "Z",
        description="ISO 8601 timestamp of when the error occurred"
    )
    details: Optional[dict] = Field(
        default=None,
        description="Additional error details for debugging"
    )

    class Config:
        schema_extra = {
            "example": {
                "error": "Invalid request parameters",
                "status_code": 400,
                "timestamp": "2025-12-28T10:30:00Z",
                "details": {
                    "field": "message",
                    "reason": "Message cannot be empty"
                }
            }
        }