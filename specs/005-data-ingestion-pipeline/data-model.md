# Data Model: Data Ingestion Pipeline

## Entity: Document Chunk
**Description**: A segment of content extracted from documentation URLs, processed for vector storage
- **Fields**:
  - `id` (string): Unique identifier for the chunk
  - `content` (string): The text content of the chunk (512 tokens with 10% overlap)
  - `source_url` (string): Original URL where the content was found
  - `title` (string): Title of the document/page where content originated
  - `embedding` (list[float]): 1024-dimensional vector representation of content
  - `metadata` (dict): Additional information including source URL, title, and processing timestamp

**Validation Rules**:
- Content must not exceed token limits for embedding model
- Source URL must be a valid URL
- Embedding must be exactly 1024 dimensions

## Entity: Embedding Result
**Description**: The vector representation and metadata of a document chunk
- **Fields**:
  - `vector_id` (string): Unique identifier in the vector database
  - `vector` (list[float]): 1024-dimensional embedding vector
  - `payload` (dict): Metadata including source_url, title, content
  - `collection_name` (string): Name of the Qdrant collection

## Entity: Processing Batch
**Description**: A group of document chunks processed together to respect API limits
- **Fields**:
  - `batch_id` (string): Unique identifier for the batch
  - `chunks` (list[Document Chunk]): Up to 90 document chunks per batch
  - `batch_size` (int): Number of chunks in the batch (max 90 for Cohere Free Tier)
  - `processing_timestamp` (datetime): When the batch was processed

## Entity: Ingestion Job
**Description**: A complete ingestion process for a set of URLs
- **Fields**:
  - `job_id` (string): Unique identifier for the ingestion job
  - `source_urls` (list[string]): List of URLs to process
  - `status` (string): Current status (pending, processing, completed, failed)
  - `start_time` (datetime): When the job started
  - `end_time` (datetime): When the job completed
  - `processed_count` (int): Number of chunks successfully processed
  - `failed_count` (int): Number of chunks that failed processing
  - `collection_name` (string): Target Qdrant collection name

## Relationships
- One Ingestion Job contains many Processing Batches
- One Processing Batch contains many Document Chunks
- One Document Chunk maps to one Embedding Result