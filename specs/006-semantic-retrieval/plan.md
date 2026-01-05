# Implementation Plan - Spec 2: Semantic Retrieval and Pipeline Verification

This plan outlines the steps to implement the retrieval logic and verify the pipeline integrity. The focus is on a single, unified file `backend/retrieve.py` that handles query embedding and vector search.

## User Stories

- As a developer, I want to query the vector database using natural language to retrieve relevant document chunks.
- As a developer, I want to verify that the 1024-dimensional embeddings generated for queries match the logic used during ingestion.

## Proposed Changes

### Backend Logic

#### [NEW] `backend/retrieve.py`

- Create a `BookRetrieval` class.
- Implement `__init__` to initialize Cohere and Qdrant clients using `backend/config.py`.
- Implement `get_query_embedding(text)`: Converts user query into a vector using `embed-english-v3.0`.
- Implement `search(query_text, top_k=5)`:
  - Generates embedding.
  - Performs similarity search in Qdrant `docusaurus_content` collection.
  - Returns a list of results containing text, score, and source URL.
- Implement a `__main__` block to act as the verification script (e.g., searching for "Isaac Sim").

## Implementation Steps

### Step 1: Initialization and Setup

- Create `backend/retrieve.py`.
- Import dependencies: `cohere`, `qdrant_client`, `dotenv`, and your local `config`.
- Set up logging to track search performance and errors.

### Step 2: Query Embedding Logic

- Implement the embedding function.
- Ensure the input is stripped of unnecessary whitespace.
- Add error handling for API failures or empty strings.

### Step 3: Vector Search Implementation

- Use the `qdrant_client.search()` method.
- Configure search parameters to return `payload` (text, URL, title).
- Ensure the search uses `Cosine` similarity as configured in Spec 1.

### Step 4: Verification Testing

- Add a test function within `retrieve.py`.
- Execute a search for a known topic from your book.
- Print results in a readable format (Score | URL | Snippet).

## Verification Plan

### Automated Tests

- Run `python backend/retrieve.py` directly.
- **Expected Output**: A list of at least 3 relevant chunks with similarity scores > 0.70.
- **Success Criteria**: The top result must match the URL and content from the ingested Docusaurus site.

### Manual Verification

- Check the Qdrant Dashboard (as verified previously) to ensure the `points_count` remains stable during read-only operations.