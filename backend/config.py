"""
Configuration module for the data ingestion pipeline and AI agent chat API.
Contains all constants and default values used in the application.
"""

import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()

# Cohere Configuration
COHERE_MODEL_NAME = "embed-english-v3.0"
COHERE_INPUT_TYPE = "search_document"
COHERE_BATCH_SIZE_LIMIT = 96  # Cohere allows up to 96 texts per request

# Qdrant Configuration
QDRANT_VECTOR_SIZE = 1024  # Cohere embeddings are 1024-dimensional
QDRANT_DISTANCE_METRIC = "Cosine"
QDRANT_COLLECTION_NAME = "docusaurus_content"

# Content Processing Configuration
DEFAULT_CHUNK_SIZE = 1000  # characters
CHUNK_OVERLAP_PERCENTAGE = 0.10  # 10% overlap
CHUNK_MIN_SIZE = 50  # minimum chunk size in characters
CHUNK_OVERLAP_SIZE = int(DEFAULT_CHUNK_SIZE * CHUNK_OVERLAP_PERCENTAGE)

# API Rate Limiting Configuration
COHERE_RATE_LIMIT_DELAY = 0.1  # seconds between requests to respect rate limits
URL_PROCESSING_DELAY = 1.0  # seconds between processing URLs

# Retry Configuration
MAX_RETRIES = 5
BASE_DELAY = 1.0  # seconds
MAX_DELAY = 60.0  # seconds
BACKOFF_FACTOR = 2.0  # exponential backoff factor

# Environment Variables
REQUIRED_ENV_VARS = [
    "COHERE_API_KEY",
    "QDRANT_URL",
    "QDRANT_API_KEY"
]

# Validation Configuration
MAX_CONTENT_SIZE = 1000000  # maximum content size in characters (1MB)
MAX_URLS_PER_BATCH = 10  # maximum number of URLs to process in one batch

# OpenRouter and AI Agent Configuration
OPENROUTER_API_KEY: Optional[str] = os.getenv("OPENROUTER_API_KEY")

# Qdrant configuration (for agent)
QDRANT_HOST: Optional[str] = os.getenv("QDRANT_HOST")
QDRANT_API_KEY_AGENT: Optional[str] = os.getenv("QDRANT_API_KEY")

# Application settings
DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"
LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

# API settings
API_HOST: str = os.getenv("API_HOST", "0.0.0.0")
API_PORT: int = int(os.getenv("API_PORT", "8000"))

# Agent settings
DEFAULT_MODEL: str = os.getenv("DEFAULT_MODEL", "openrouter/auto")


def validate_config() -> bool:
    """
    Validate that required configuration values are present
    """
    required_vars = ["OPENROUTER_API_KEY"]
    missing_vars = []

    for var in required_vars:
        value = globals().get(var)
        if not value:
            missing_vars.append(var)

    if missing_vars:
        print(f"Warning: Missing required environment variables: {', '.join(missing_vars)}")
        return False

    return True

# Validate configuration on import
config_valid = validate_config()
if not config_valid and DEBUG:
    print("Configuration validation failed - some features may not work properly")