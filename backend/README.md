# Data Ingestion Pipeline for Docusaurus Documentation

This project implements a data ingestion pipeline that extracts content from Docusaurus documentation sites, generates vector embeddings using Cohere, and stores them in Qdrant Cloud for use in RAG (Retrieval-Augmented Generation) applications.

## Features

- **Automated Content Extraction**: Extracts content from Docusaurus documentation sites using sitemap.xml discovery
- **Smart Content Chunking**: Splits content into overlapping chunks (10% overlap) to preserve context boundaries
- **Vector Embeddings**: Generates 1024-dimensional embeddings using Cohere's `embed-english-v3.0` model
- **Vector Storage**: Stores embeddings in Qdrant Cloud with metadata for semantic search
- **Resilience**: Implements exponential backoff retry logic for API calls
- **Rate Limiting**: Respects API rate limits and includes proper delays between requests
- **Progress Tracking**: Shows processing progress with URL-by-URL updates

## Prerequisites

- Python 3.8 or higher
- `uv` package manager (install with `pip install uv`)
- API keys for Cohere and Qdrant Cloud

## Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd Physical_AI_Humanoid_Robotics
```

### 2. Install Dependencies

```bash
cd backend
uv init
uv add cohere qdrant-client beautifulsoup4 python-dotenv requests
```

### 3. Configure Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
# Cohere API Key for generating embeddings
COHERE_API_KEY="your-cohere-api-key-here"

# Qdrant Cloud URL and API Key for vector storage
QDRANT_URL="your-qdrant-cloud-url-here"
QDRANT_API_KEY="your-qdrant-api-key-here"

# Vercel Deployment URL (or your Docusaurus site URL)
DEPLOY_VERCEL_URL="https://your-site-url.vercel.app/"
```

**Note**: Never commit your `.env` file to version control. The `.env.example` file is provided for reference.

## Usage

### Run the Ingestion Pipeline

To process a Docusaurus site and store embeddings in Qdrant:

```bash
uv run python main.py --url https://your-docusaurus-site.com
```

### Dry Run

To perform a dry run that estimates chunk counts without processing content:

```bash
uv run python main.py --url https://your-docusaurus-site.com --dry-run
```

The dry run will:
- Fetch and parse the sitemap.xml
- Discover all documentation URLs
- Estimate the number of chunks for each URL
- Show the total estimated chunks across all URLs
- Exit without calling the Cohere API

## Configuration

The pipeline behavior can be customized through the `config.py` file:

- `DEFAULT_CHUNK_SIZE`: Default size for text chunks (default: 1000 characters)
- `CHUNK_OVERLAP_PERCENTAGE`: Overlap percentage between chunks (default: 0.10 for 10%)
- `COHERE_MODEL_NAME`: Cohere model to use (default: "embed-english-v3.0")
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: "docusaurus_content")
- `MAX_RETRIES`: Maximum number of retry attempts for failed API calls
- And more configuration options...

## How It Works

1. **Sitemap Discovery**: The pipeline fetches and parses the `sitemap.xml` from the provided URL to discover all documentation pages.

2. **Content Extraction**: For each URL, the pipeline extracts the main content using BeautifulSoup, focusing on common Docusaurus content selectors.

3. **Content Chunking**: The extracted content is split into overlapping chunks using a sliding window approach with configurable overlap percentage to preserve context boundaries.

4. **Embedding Generation**: Each chunk is sent to Cohere's embedding API to generate 1024-dimensional vector embeddings.

5. **Vector Storage**: The embeddings and their associated metadata (title, URL, content) are stored in Qdrant Cloud for efficient semantic search.

6. **Progress Tracking**: The pipeline provides real-time progress updates showing which URL is currently being processed.

## Error Handling and Resilience

- **Retry Logic**: Implements exponential backoff with configurable parameters for API failures
- **Rate Limiting**: Respects API rate limits with configurable delays between requests
- **Connection Handling**: Gracefully handles connection failures to external services
- **Comprehensive Logging**: Detailed logging for monitoring and debugging
- **Graceful Degradation**: Continues processing other URLs if one fails

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your Cohere and Qdrant API keys are valid and have the necessary permissions.

2. **Rate Limiting**: If you encounter rate limit errors, the pipeline includes built-in delays, but you may need to adjust the configuration.

3. **Sitemap Not Found**: Verify that the target Docusaurus site has a valid `sitemap.xml` file accessible at `/sitemap.xml`.

4. **Content Extraction Issues**: The pipeline uses common selectors for Docusaurus content. If content extraction fails for a specific site, you may need to adjust the selectors in the `extract_content_from_url` method.

### Logging

The pipeline logs all activities at the INFO level. Check the logs for detailed information about the processing status, errors, and warnings.

## Project Structure

```
backend/
├── main.py              # Main pipeline implementation
├── config.py            # Configuration constants
├── utils.py             # Utility functions (retry logic)
├── .env.example         # Example environment variables
└── README.md            # This file
```

## Performance Considerations

- The pipeline processes URLs sequentially to respect rate limits
- Cohere API calls are batched to optimize performance while respecting limits
- Qdrant upserts are batched for efficiency
- Progress tracking helps monitor long-running operations

## Security

- API keys are loaded from environment variables, not hardcoded
- No secrets are stored in the codebase
- Follows best practices for handling sensitive information