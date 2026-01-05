from typing import Optional, Dict, Any
from fastapi import HTTPException
from datetime import datetime
from ..models.chat import APIError


class APIErrorHandler:
    """
    Utility class for handling API errors and creating consistent error responses
    """

    @staticmethod
    def create_error_response(
        error_message: str,
        status_code: int,
        details: Optional[Dict[str, Any]] = None
    ) -> APIError:
        """
        Create a standardized API error response
        """
        return APIError(
            error=error_message,
            status_code=status_code,
            timestamp=datetime.utcnow().isoformat() + "Z",
            details=details
        )

    @staticmethod
    def handle_validation_error(
        field_name: str,
        reason: str,
        status_code: int = 400
    ) -> APIError:
        """
        Create an error response for validation failures
        """
        details = {
            "field": field_name,
            "reason": reason
        }
        return APIErrorHandler.create_error_response(
            error_message="Invalid request parameters",
            status_code=status_code,
            details=details
        )

    @staticmethod
    def handle_internal_error(
        error_type: str,
        message: str,
        status_code: int = 500
    ) -> APIError:
        """
        Create an error response for internal server errors
        """
        details = {
            "type": error_type,
            "message": message
        }
        return APIErrorHandler.create_error_response(
            error_message="Internal server error occurred while processing the request",
            status_code=status_code,
            details=details
        )


def raise_http_exception(
    status_code: int,
    detail: str,
    headers: Optional[Dict[str, str]] = None
) -> None:
    """
    Convenience function to raise HTTPException with proper error handling
    """
    raise HTTPException(
        status_code=status_code,
        detail=detail,
        headers=headers
    )


def handle_agent_error(error: Exception) -> APIError:
    """
    Handle errors from the AI agent and create appropriate APIError response
    """
    error_type = type(error).__name__
    error_message = str(error)

    # Log the error (in a real implementation, this would go to a proper logger)
    print(f"Agent error: {error_type} - {error_message}")

    return APIErrorHandler.handle_internal_error(
        error_type=error_type,
        message=error_message
    )