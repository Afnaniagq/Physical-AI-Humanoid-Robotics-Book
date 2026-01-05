# Quickstart: Data Ingestion Pipeline

## Prerequisites
- Python 3.11 or higher
- uv package manager
- Access to Cohere API (Free Tier)
- Qdrant Cloud account (Free Tier)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd Physical_AI_Humanoid_Robotics
```

### 2. Navigate to Backend Directory
```bash
cd backend
```

### 3. Initialize the Project
```bash
uv init
```

### 4. Install Dependencies
```bash
uv add cohere qdrant-client beautifulsoup4 python-dotenv requests
```

### 5. Set Up Environment Variables
Copy the example environment file:
```bash
cp .env.example .env
```

Edit `.env` and add your API keys:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
```

## Running the Ingestion Pipeline

### Basic Usage
```bash
python main.py
```

### With Custom Parameters
The main.py script can be configured to process different URLs and collections:

```bash
# Example of how to customize the ingestion
python main.py --urls "https://docs.example.com" --collection "documentation_chunks"
```

## Configuration Options

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key for generating embeddings
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `CHUNK_SIZE`: Size of text chunks in tokens (default: 512)
- `OVERLAP_PERCENTAGE`: Overlap between chunks as percentage (default: 10%)
- `BATCH_SIZE`: Number of chunks per embedding batch (max 90 for Free Tier)

### Rate Limiting
The pipeline automatically respects Cohere Free Tier limits by:
- Limiting batches to 90 chunks per API call
- Adding 1-second delays between batch requests
- Implementing retry logic with exponential backoff for failed requests

## Example Usage in Code
The ingestion pipeline performs these steps automatically:
1. Extract content from specified URLs using BeautifulSoup
2. Chunk the content into 512-token segments with 10% overlap
3. Generate embeddings using Cohere's embed-english-v3.0 model
4. Store the embeddings and metadata in Qdrant Cloud
5. Verify the stored count matches expected values