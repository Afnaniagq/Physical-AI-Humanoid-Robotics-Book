from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


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