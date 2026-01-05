# API Contract: Data Ingestion Pipeline

## Overview
This document describes the interface and contract for the data ingestion pipeline that extracts content from documentation URLs, generates embeddings, and stores them in Qdrant.

## Configuration Interface

### Environment Variables Contract
The ingestion pipeline expects the following environment variables:

```
COHERE_API_KEY: string (required)
  - Purpose: API key for Cohere embedding service
  - Format: Standard API key format
  - Validation: Must be a non-empty string

QDRANT_API_KEY: string (required)
  - Purpose: API key for Qdrant Cloud service
  - Format: Standard API key format
  - Validation: Must be a non-empty string

QDRANT_URL: string (required)
  - Purpose: URL for Qdrant Cloud cluster
  - Format: Valid HTTPS URL
  - Validation: Must be a properly formatted URL

CHUNK_SIZE: integer (optional, default: 512)
  - Purpose: Size of text chunks in tokens
  - Format: Positive integer
  - Validation: Must be between 100 and 1024

OVERLAP_PERCENTAGE: integer (optional, default: 10)
  - Purpose: Percentage overlap between chunks
  - Format: Integer between 0 and 50
  - Validation: Must be between 0 and 50

BATCH_SIZE: integer (optional, default: 90)
  - Purpose: Number of chunks per embedding batch
  - Format: Positive integer
  - Validation: Must be between 1 and 90 (to respect Free Tier limits)
```

## Function Interface Contracts

### extract_content_from_url(url: str) -> str
**Purpose**: Extract text content from a given URL
- **Input**: Valid URL string
- **Output**: Extracted text content as string
- **Errors**:
  - NetworkError: When URL is inaccessible
  - ParseError: When content cannot be parsed

### chunk_text(text: str, chunk_size: int, overlap_percentage: int) -> List[str]
**Purpose**: Split text into overlapping chunks
- **Input**:
  - text: Content to be chunked
  - chunk_size: Size of each chunk in tokens
  - overlap_percentage: Percentage overlap between chunks
- **Output**: List of text chunks
- **Errors**: None

### generate_embeddings(chunks: List[str]) -> List[List[float]]
**Purpose**: Generate embeddings for text chunks using Cohere
- **Input**: List of text chunks (max 90 per call)
- **Output**: List of embedding vectors (each 1024 dimensions)
- **Errors**:
  - RateLimitError: When API rate limits are exceeded
  - AuthenticationError: When API key is invalid

### store_embeddings(embeddings: List[EmbeddingResult], collection_name: str) -> bool
**Purpose**: Store embeddings in Qdrant collection
- **Input**:
  - embeddings: List of embedding results with metadata
  - collection_name: Name of target Qdrant collection
- **Output**: Success status (true/false)
- **Errors**:
  - ConnectionError: When unable to connect to Qdrant
  - StorageError: When storage operation fails

## Data Contract

### Embedding Result Structure
```
{
  "vector_id": string (unique identifier),
  "vector": [float] (1024-dimensional array),
  "payload": {
    "content": string (original chunk text),
    "source_url": string (original URL),
    "title": string (document title),
    "created_at": string (ISO timestamp)
  }
}
```

## Error Handling Contract

The ingestion pipeline will:
1. Log all errors with appropriate severity levels
2. Implement retry logic with exponential backoff for transient failures
3. Continue processing when possible after individual chunk failures
4. Provide a summary of successful and failed operations
5. Respect rate limits by implementing appropriate delays

## Performance Contract

The pipeline guarantees:
- Processing will respect Cohere Free Tier rate limits (max 90 embeddings per call)
- 1-second delays between batch requests to avoid rate limiting
- Proper handling of large documents by chunking them appropriately
- Verification that stored count matches expected number of chunks