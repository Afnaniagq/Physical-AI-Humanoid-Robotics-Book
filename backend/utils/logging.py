import logging
from datetime import datetime
from typing import Optional


class ChatLogger:
    """
    Utility class for logging chat interactions and system events
    """

    def __init__(self, name: str = "chatbot_api"):
        self.logger = logging.getLogger(name)

        # Set up basic configuration if not already configured
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)

    def log_chat_request(self, session_id: str, message: str, ip_address: Optional[str] = None):
        """
        Log incoming chat requests
        """
        self.logger.info(
            f"Chat request - Session: {session_id}, IP: {ip_address or 'unknown'}, "
            f"Message: {message[:100]}{'...' if len(message) > 100 else ''}"
        )

    def log_chat_response(self, session_id: str, response: str, processing_time: Optional[float] = None):
        """
        Log chat responses
        """
        time_info = f", Processing time: {processing_time:.2f}s" if processing_time else ""
        self.logger.info(
            f"Chat response - Session: {session_id}, "
            f"Response length: {len(response)} characters{time_info}"
        )

    def log_error(self, session_id: str, error_message: str, error_type: str = "general"):
        """
        Log errors that occur during chat processing
        """
        self.logger.error(
            f"Chat error - Session: {session_id}, Type: {error_type}, Error: {error_message}"
        )

    def log_system_event(self, event_type: str, details: str):
        """
        Log system events
        """
        self.logger.info(f"System event - Type: {event_type}, Details: {details}")


# Create a global logger instance
chat_logger = ChatLogger()


def get_logger() -> ChatLogger:
    """
    Get the global chat logger instance
    """
    return chat_logger