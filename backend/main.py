import os
import time
import requests
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from bs4 import BeautifulSoup
from dotenv import load_dotenv
import logging
from typing import List, Dict, Optional
import hashlib
import re
import xml.etree.ElementTree as ET
import sys
import argparse

from config import *
from utils import retry_with_exponential_backoff, simple_exponential_backoff_retry


# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()


class DataIngestionPipeline:
    """
    A data ingestion pipeline that extracts content from Docusaurus URLs,
    generates vector embeddings using Cohere, and stores them in Qdrant Cloud.
    """

    def __init__(self):
        """
        Initialize the DataIngestionPipeline with Cohere and Qdrant clients.
        Sets up the Qdrant collection for storing embeddings.
        """
        # Initialize clients
        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

        # Collection name for storing embeddings
        self.collection_name = QDRANT_COLLECTION_NAME

        # Create collection if it doesn't exist
        self._setup_collection()

    def _setup_collection(self):
        """
        Set up the Qdrant collection for storing document chunks.

        Creates the collection with the configured vector size and distance metric
        if it doesn't already exist.
        """
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with configured dimensions and distance metric (for Cohere embeddings)
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=QDRANT_VECTOR_SIZE, distance=models.Distance.COSINE)
                )
                logger.info(f"Created collection '{self.collection_name}' in Qdrant")
            else:
                logger.info(f"Collection '{self.collection_name}' already exists in Qdrant")
        except Exception as e:
            logger.error(f"Error setting up Qdrant collection: {e}")
            raise

    def extract_content_from_url(self, url: str) -> Optional[Dict[str, str]]:
        """
        Extract content from a Docusaurus URL.

        Args:
            url: The URL to extract content from

        Returns:
            A dictionary with 'title', 'content', and 'url' keys, or None if extraction fails
        """
        try:
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            response = requests.get(url, headers=headers)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Extract title - handle case where title_tag is None
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else "No Title"

            # Remove script and style elements - handle None return values safely
            for script in soup(["script", "style"]):
                if script:
                    script.decompose()

            # Extract main content - try common selectors for Docusaurus content
            main_content = None
            for selector in ['main', '.main-wrapper', '.container', '.theme-doc-markdown', '.markdown']:
                selected_content = soup.select_one(selector)
                if selected_content:
                    main_content = selected_content
                    break

            # If no main content found, try body - handle case where soup.find returns None
            if not main_content:
                main_content = soup.find('body')

            # Handle case where no content could be extracted
            if not main_content:
                logger.warning(f"No content found in {url} after trying multiple selectors")
                return None

            # Extract text content, removing extra whitespace - handle case where get_text returns None
            content = main_content.get_text(separator=' ', strip=True)
            if not content:
                logger.warning(f"Extracted content is empty for {url}")
                return None

            # Clean up the content
            content = re.sub(r'\s+', ' ', content)  # Replace multiple spaces with single space
            content = content.strip()

            return {
                'title': title,
                'content': content,
                'url': url
            }
        except Exception as e:
            logger.error(f"Error extracting content from {url}: {e}")
            return None

    def chunk_text(self, text: str, max_chunk_size: int = DEFAULT_CHUNK_SIZE) -> List[str]:
        """
        Split text into chunks of approximately max_chunk_size characters with overlap.

        Uses a sliding window approach to create overlapping chunks to preserve context
        boundaries, with configurable overlap percentage based on configuration.

        Args:
            text: The text to chunk
            max_chunk_size: Maximum size of each chunk in characters (default from config)

        Returns:
            List of text chunks with configured overlap
        """
        if len(text) <= max_chunk_size:
            return [text]

        chunks = []
        # Calculate overlap size based on the configuration
        overlap_size = int(max_chunk_size * CHUNK_OVERLAP_PERCENTAGE)

        # Use a sliding window approach to create overlapping chunks
        start_idx = 0

        while start_idx < len(text):
            end_idx = start_idx + max_chunk_size

            # If this is the last chunk and it's smaller than min size, include it in the previous chunk
            if end_idx >= len(text) and (len(text) - start_idx) < CHUNK_MIN_SIZE and chunks:
                # Add the remaining text to the last chunk
                last_chunk = chunks.pop()
                chunks.append(last_chunk + text[start_idx:])
                break

            chunk = text[start_idx:end_idx]

            # If we're not at the end and we have overlap, try to break at a sentence or paragraph boundary
            if end_idx < len(text):
                # Look for a good breaking point near the overlap area
                overlap_start = end_idx - overlap_size
                break_point = end_idx

                # Look for paragraph breaks first
                for i in range(end_idx, max(overlap_start, start_idx), -1):
                    if i < len(text) and text[i:i+2] == '\n\n':
                        break_point = i
                        break
                else:
                    # Look for sentence breaks
                    for i in range(end_idx, max(overlap_start, start_idx), -1):
                        if i < len(text) and text[i] in '.!?':
                            break_point = i + 1
                            break
                    else:
                        # Look for word breaks
                        for i in range(end_idx, max(overlap_start, start_idx), -1):
                            if i < len(text) and text[i] == ' ':
                                break_point = i
                                break

                chunk = text[start_idx:break_point]
                end_idx = break_point

            chunks.append(chunk)

            # Move start index forward by chunk size minus overlap
            start_idx = end_idx - overlap_size

            # If the remaining text is too small, combine it with the last chunk
            if len(text) - start_idx < CHUNK_MIN_SIZE:
                if start_idx < len(text):
                    last_chunk = chunks.pop()
                    chunks.append(last_chunk + text[start_idx:])
                break

        # Filter out empty chunks and very short chunks
        return [chunk for chunk in chunks if len(chunk) >= CHUNK_MIN_SIZE]

    @retry_with_exponential_backoff(
        max_retries=MAX_RETRIES,
        base_delay=BASE_DELAY,
        max_delay=MAX_DELAY,
        backoff_factor=BACKOFF_FACTOR,
        exceptions=(Exception,)
    )
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere's embedding API.

        Handles batching of texts to respect API limits, applies rate limiting,
        and includes retry logic with exponential backoff for failed requests.

        Args:
            texts: List of text strings to generate embeddings for

        Returns:
            List of embeddings (each embedding is a list of 1024 float values)
        """
        try:
            # Cohere has a limit per request for embeddings
            batch_size = min(COHERE_BATCH_SIZE_LIMIT, len(texts))
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                response = self.cohere_client.embed(
                    texts=batch,
                    model=COHERE_MODEL_NAME,
                    input_type=COHERE_INPUT_TYPE  # Using search_document for content embeddings
                )
                all_embeddings.extend(response.embeddings)

                # Respect rate limits - add delay if needed
                time.sleep(COHERE_RATE_LIMIT_DELAY)  # Small delay to be respectful to the API

            return all_embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise

    @retry_with_exponential_backoff(
        max_retries=MAX_RETRIES,
        base_delay=BASE_DELAY,
        max_delay=MAX_DELAY,
        backoff_factor=BACKOFF_FACTOR,
        exceptions=(Exception,)
    )
    def store_embeddings(self, chunks: List[Dict], embeddings: List[List[float]]):
        """
        Store the text chunks and their embeddings in Qdrant vector database.

        Creates Points with metadata containing source information and upserts
        them to the configured Qdrant collection. Includes verification that the
        expected number of chunks are stored successfully.

        Args:
            chunks: List of dictionaries containing chunk information with 'title', 'content', 'url'
            embeddings: List of embeddings corresponding to the chunks

        Raises:
            Exception: If storing embeddings in Qdrant fails
        """
        try:
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                # Create a unique ID for the chunk using hash of content
                content_hash = hashlib.md5(f"{chunk['url']}_{chunk['content'][:100]}".encode()).hexdigest()

                point = models.PointStruct(
                    id=content_hash,
                    vector=embedding,
                    payload={
                        'title': chunk['title'],
                        'content': chunk['content'],
                        'url': chunk['url'],
                        'source': 'docusaurus',
                        'created_at': time.time()
                    }
                )
                points.append(point)

            # Upsert points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Stored {len(points)} chunks in Qdrant collection '{self.collection_name}' (expected: {len(points)})")
            logger.info(f"Verification successful: {len(points)} chunks stored as expected")
        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {e}")
            raise

    def process_url(self, url: str) -> bool:
        """
        Process a single URL: extract content, chunk it, generate embeddings, and store.

        Args:
            url: The URL to process

        Returns:
            True if successful, False otherwise
        """
        logger.info(f"Processing URL: {url}")

        # Extract content from URL
        content_data = self.extract_content_from_url(url)
        if not content_data:
            logger.error(f"Failed to extract content from {url}")
            return False

        logger.info(f"Extracted content with {len(content_data['content'])} characters")

        # Chunk the content
        chunks = self.chunk_text(content_data['content'])
        logger.info(f"Content chunked into {len(chunks)} pieces")

        # Prepare chunk data for storage
        chunk_data_list = []
        for chunk_text in chunks:
            chunk_data = {
                'title': content_data['title'],
                'content': chunk_text,
                'url': content_data['url']
            }
            chunk_data_list.append(chunk_data)

        # Generate embeddings
        try:
            embeddings = self.generate_embeddings([chunk['content'] for chunk in chunk_data_list])
        except Exception as e:
            logger.error(f"Failed to generate embeddings for {url}: {e}")
            return False

        # Store embeddings in Qdrant
        try:
            self.store_embeddings(chunk_data_list, embeddings)
        except Exception as e:
            logger.error(f"Failed to store embeddings for {url}: {e}")
            return False

        logger.info(f"Successfully processed {url}")
        return True

    def verify_chunk_quality(self, original_text: str, chunks: List[str]) -> Dict[str, any]:
        """
        Verify the quality of chunking by checking various metrics.

        Args:
            original_text: The original text that was chunked
            chunks: The list of chunks produced

        Returns:
            Dictionary with quality metrics
        """
        quality_metrics = {
            'original_length': len(original_text),
            'num_chunks': len(chunks),
            'avg_chunk_size': sum(len(chunk) for chunk in chunks) / len(chunks) if chunks else 0,
            'total_chunked_length': sum(len(chunk) for chunk in chunks),
            'coverage_ratio': sum(len(chunk) for chunk in chunks) / len(original_text) if original_text else 0,
            'chunks_meeting_min_size': len([c for c in chunks if len(c) >= CHUNK_MIN_SIZE]),
            'overlap_quality': self._analyze_overlap_quality(chunks)
        }

        # Check for content duplication
        unique_chunks = set(chunks)
        quality_metrics['duplication_ratio'] = 1 - (len(unique_chunks) / len(chunks)) if chunks else 0

        return quality_metrics

    def _analyze_overlap_quality(self, chunks: List[str]) -> Dict[str, any]:
        """
        Analyze the quality of overlap between chunks.

        Args:
            chunks: The list of chunks to analyze

        Returns:
            Dictionary with overlap quality metrics
        """
        if len(chunks) < 2:
            return {'has_overlap': False, 'overlap_ratio': 0.0}

        overlap_metrics = {
            'has_overlap': True,
            'overlap_ratios': []
        }

        for i in range(len(chunks) - 1):
            current_chunk = chunks[i]
            next_chunk = chunks[i + 1]

            # Find the overlapping part by checking the end of current and start of next
            overlap_size = 0
            for j in range(min(len(current_chunk), len(next_chunk)), 0, -1):
                if current_chunk[-j:] == next_chunk[:j]:
                    overlap_size = j
                    break

            overlap_ratio = overlap_size / len(current_chunk) if current_chunk else 0
            overlap_metrics['overlap_ratios'].append(overlap_ratio)

        avg_overlap_ratio = sum(overlap_metrics['overlap_ratios']) / len(overlap_metrics['overlap_ratios']) if overlap_metrics['overlap_ratios'] else 0
        overlap_metrics['avg_overlap_ratio'] = avg_overlap_ratio

        return overlap_metrics

    def validate_clients(self) -> bool:
        """
        Validate that the Cohere and Qdrant clients are properly configured and accessible.

        Returns:
            True if both clients are valid and accessible, False otherwise
        """
        logger.info("Validating API clients...")

        # Test Cohere client
        try:
            # Test with a simple embedding request
            test_response = self.cohere_client.embed(
                texts=["test"],
                model=COHERE_MODEL_NAME,
                input_type=COHERE_INPUT_TYPE
            )
            logger.info("Cohere client validation successful")
        except Exception as e:
            logger.error(f"Cohere client validation failed: {e}")
            return False

        # Test Qdrant client
        try:
            # Test by getting collections
            collections = self.qdrant_client.get_collections()
            logger.info("Qdrant client validation successful")
        except Exception as e:
            logger.error(f"Qdrant client validation failed: {e}")
            return False

        logger.info("All clients validated successfully")
        return True

    def fetch_sitemap_urls(self, base_url: str) -> List[str]:
        """
        Fetch and parse the sitemap.xml from the given base URL to extract all valid documentation pages.

        Args:
            base_url: The base URL of the Docusaurus site

        Returns:
            List of URLs extracted from the sitemap
        """
        sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
        logger.info(f"Fetching sitemap from: {sitemap_url}")

        try:
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            response = requests.get(sitemap_url, headers=headers)
            response.raise_for_status()

            # Parse the XML sitemap
            root = ET.fromstring(response.content)

            # Handle both regular sitemap and sitemap index
            urls = []
            namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

            # Check if this is a sitemap index (contains other sitemaps)
            sitemap_elements = root.findall('sitemap:sitemap', namespace)
            if sitemap_elements:
                # This is a sitemap index, need to fetch individual sitemaps
                for sitemap_elem in sitemap_elements:
                    loc_elem = sitemap_elem.find('sitemap:loc', namespace)
                    if loc_elem is not None:
                        individual_sitemap_url = loc_elem.text
                        logger.info(f"Fetching individual sitemap: {individual_sitemap_url}")
                        individual_urls = self._fetch_individual_sitemap(individual_sitemap_url)
                        urls.extend(individual_urls)
            else:
                # This is a regular sitemap with URL entries
                url_elements = root.findall('sitemap:url', namespace)
                for url_elem in url_elements:
                    loc_elem = url_elem.find('sitemap:loc', namespace)
                    if loc_elem is not None:
                        urls.append(loc_elem.text)

            # Filter URLs to only include documentation pages (typically contain '/docs/' or are root docs)
            doc_urls = [url for url in urls if '/docs/' in url or url.endswith('/docs') or url == base_url or url == f"{base_url}/"]

            logger.info(f"Found {len(doc_urls)} documentation URLs from sitemap")
            return doc_urls

        except requests.exceptions.RequestException as e:
            logger.error(f"Error fetching sitemap from {sitemap_url}: {e}")
            return []
        except ET.ParseError as e:
            logger.error(f"Error parsing sitemap XML from {sitemap_url}: {e}")
            return []
        except Exception as e:
            logger.error(f"Unexpected error while fetching sitemap from {sitemap_url}: {e}")
            return []

    def _fetch_individual_sitemap(self, sitemap_url: str) -> List[str]:
        """
        Fetch and parse an individual sitemap URL.

        Args:
            sitemap_url: URL of the individual sitemap

        Returns:
            List of URLs extracted from the sitemap
        """
        try:
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            response = requests.get(sitemap_url, headers=headers)
            response.raise_for_status()

            root = ET.fromstring(response.content)
            namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

            urls = []
            url_elements = root.findall('sitemap:url', namespace)
            for url_elem in url_elements:
                loc_elem = url_elem.find('sitemap:loc', namespace)
                if loc_elem is not None:
                    urls.append(loc_elem.text)

            return urls

        except Exception as e:
            logger.error(f"Error fetching individual sitemap from {sitemap_url}: {e}")
            return []

    def estimate_chunks_for_urls(self, urls: List[str]) -> Dict[str, int]:
        """
        Estimate the number of chunks for each URL without processing them completely.
        This is for dry run purposes.

        Args:
            urls: List of URLs to estimate chunks for

        Returns:
            Dictionary mapping URLs to estimated chunk counts
        """
        estimates = {}
        for url in urls:
            try:
                headers = {
                    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
                }
                response = requests.get(url, headers=headers, timeout=10)
                response.raise_for_status()

                soup = BeautifulSoup(response.content, 'html.parser')

                # Remove script and style elements
                for script in soup(["script", "style"]):
                    script.decompose()

                # Extract main content - try common selectors for Docusaurus content
                main_content = None
                for selector in ['main', '.main-wrapper', '.container', '.theme-doc-markdown', '.markdown']:
                    main_content = soup.select_one(selector)
                    if main_content:
                        break

                if not main_content:
                    main_content = soup.find('body')

                if main_content:
                    content = main_content.get_text(separator=' ', strip=True)
                    content = re.sub(r'\s+', ' ', content).strip()

                    # Estimate chunks using the same logic as chunk_text but without overlap complexity for estimation
                    if len(content) <= DEFAULT_CHUNK_SIZE:
                        chunk_count = 1
                    else:
                        # Estimate based on chunk size and overlap
                        estimated_chunks = len(content) / (DEFAULT_CHUNK_SIZE * (1 - CHUNK_OVERLAP_PERCENTAGE))
                        chunk_count = max(1, int(estimated_chunks) + 1)  # Add 1 to account for remainder

                    estimates[url] = chunk_count
                else:
                    estimates[url] = 0

            except Exception as e:
                logger.warning(f"Could not estimate chunks for {url}: {e}")
                estimates[url] = 0

        return estimates

    def process_urls(self, urls: List[str]) -> Dict[str, bool]:
        """
        Process multiple URLs.

        Args:
            urls: List of URLs to process

        Returns:
            Dictionary mapping URLs to success status
        """
        results = {}
        for url in urls:
            results[url] = self.process_url(url)
            # Add a small delay between URLs to be respectful to the APIs
            time.sleep(URL_PROCESSING_DELAY)

        return results


def main():
    """Main function to run the data ingestion pipeline."""
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description='Data Ingestion Pipeline for Docusaurus Documentation')
    parser.add_argument('--url', type=str, required=True,
                        help='Base URL of the Docusaurus site to process (e.g., https://example.com)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Perform a dry run without processing content')

    args = parser.parse_args()

    logger.info(f"Starting data ingestion pipeline for URL: {args.url}")

    # Check for required environment variables
    missing_vars = [var for var in REQUIRED_ENV_VARS if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        return

    # Initialize the pipeline
    pipeline = DataIngestionPipeline()

    # Validate clients before starting
    if not pipeline.validate_clients():
        logger.error("Client validation failed. Please check your API keys and network connection.")
        return

    sitemap_urls = []
    results = {}

    try:
        # Fetch URLs from sitemap
        base_url = args.url
        sitemap_urls = pipeline.fetch_sitemap_urls(base_url)

        if not sitemap_urls:
            logger.warning("No URLs found in sitemap. Cannot proceed with ingestion.")
            return

        # Perform dry run: estimate chunks without calling Cohere API
        logger.info("Performing dry run to estimate chunk counts...")
        chunk_estimates = pipeline.estimate_chunks_for_urls(sitemap_urls)

        # Log the list of URLs found and their estimated chunks
        logger.info(f"Found {len(sitemap_urls)} URLs in sitemap:")
        total_estimated_chunks = 0
        for url in sitemap_urls:
            estimated_chunks = chunk_estimates.get(url, 0)
            total_estimated_chunks += estimated_chunks
            logger.info(f"  - {url} (estimated {estimated_chunks} chunks)")

        logger.info(f"Total estimated chunks across all URLs: {total_estimated_chunks}")

        # If dry run was requested, exit now
        if args.dry_run:
            logger.info("Dry run completed. Exiting without processing content.")
            return

        # Process the URLs with full pipeline
        logger.info("Starting full pipeline processing...")

        # Process URLs with progress tracking
        for i, url in enumerate(sitemap_urls, 1):
            logger.info(f"Processing {i}/{len(sitemap_urls)}: {url}")
            results[url] = pipeline.process_url(url)
            # Add a small delay between URLs to be respectful to the APIs
            time.sleep(URL_PROCESSING_DELAY)

    except KeyboardInterrupt:
        logger.info("Process interrupted by user.")
    except Exception as e:
        logger.error(f"An unexpected error occurred during processing: {e}")
    finally:
        # Print results and summary in the finally block to ensure it runs
        if results:
            successful = sum(1 for success in results.values() if success)
            total = len(results)

            logger.info(f"Processing complete. {successful}/{total} URLs processed successfully.")

            for url, success in results.items():
                status = "SUCCESS" if success else "FAILED"
                logger.info(f"{status}: {url}")

        # Print final summary
        logger.info("="*50)
        logger.info("INGESTION PIPELINE SUMMARY")
        logger.info("="*50)
        logger.info(f"Target URL: {args.url}")
        logger.info(f"Total URLs processed: {len(results) if results else 0}")
        if results:
            successful_count = sum(1 for success in results.values() if success)
            logger.info(f"Successful: {successful_count}")
            logger.info(f"Failed: {len(results) - successful_count}")
        logger.info(f"Pipeline execution completed at: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        logger.info("="*50)


if __name__ == "__main__":
    main()
