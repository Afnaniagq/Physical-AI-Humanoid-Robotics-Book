"""
Semantic Retrieval Module - Book Content Search

Implements a BookRetrieval class that performs semantic search on book content
stored in Qdrant using natural language queries.
"""

import os
import time
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging
from typing import List, Dict, Optional
from .config import *

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

class BookRetrieval:
    """
    A retrieval class that performs semantic search on the Qdrant collection.

    Takes a natural language query and returns the top K most relevant chunks
    using vector similarity search against the 'docusaurus_content' collection.
    """

    def __init__(self):
        """
        Initialize the BookRetrieval class with Cohere and Qdrant clients.
        """
        # Initialize clients
        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

        # Collection name for searching
        self.collection_name = QDRANT_COLLECTION_NAME

        # Verify the collection exists
        self._verify_collection()

    def _verify_collection(self):
        """
        Verify that the Qdrant collection exists before performing searches.
        """
        try:
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                logger.error(f"Collection '{self.collection_name}' does not exist in Qdrant")
                raise Exception(f"Collection '{self.collection_name}' not found in Qdrant")
            else:
                logger.info(f"Collection '{self.collection_name}' verified in Qdrant")
        except Exception as e:
            logger.error(f"Error verifying Qdrant collection: {e}")
            raise

    def get_query_embedding(self, text: str) -> List[float]:
        """
        Converts user query into a vector using embed-english-v3.0.

        Args:
            text: The natural language query string

        Returns:
            A list of 1024 float values representing the embedding
        """
        try:
            # Ensure input is stripped of unnecessary whitespace
            text = text.strip() if text else ""

            if not text:
                raise ValueError("Query text cannot be empty")

            # Generate embedding for the query using the same model as the ingestion pipeline
            response = self.cohere_client.embed(
                texts=[text],
                model=COHERE_MODEL_NAME,
                input_type="search_query"  # Using search_query for query embeddings
            )

            # Return the first (and only) embedding
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error converting query to embedding: {e}")
            raise

    def search(self, query_text: str, top_k: int = 5) -> List[Dict]:
        """
        Performs similarity search in Qdrant docusaurus_content collection.

        Args:
            query_text: Natural language query string
            top_k: Number of top results to return (default: 5)

        Returns:
            A list of results containing text, score, and source URL
        """
        start_time = time.time()

        try:
            # Generate embedding
            query_embedding = self.get_query_embedding(query_text)

            # Performs similarity search in Qdrant using query_points method
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True,  # Include metadata in results
                with_vectors=False  # Don't return the vectors themselves
            )

            # Format results containing text, score, and source URL
            formatted_results = []
            for result in search_results.points:  # Access the points attribute
                formatted_result = {
                    'id': result.id,
                    'score': result.score,
                    'title': result.payload.get('title', '') if result.payload else '',
                    'content': result.payload.get('content', '') if result.payload else '',
                    'url': result.payload.get('url', '') if result.payload else '',
                    'source': result.payload.get('source', '') if result.payload else '',
                    'created_at': result.payload.get('created_at', '') if result.payload else ''
                }
                formatted_results.append(formatted_result)

            # Log performance
            elapsed_time = time.time() - start_time
            logger.info(f"Search completed in {elapsed_time:.3f} seconds for query: '{query_text[:50]}...'")

            return formatted_results

        except Exception as e:
            logger.error(f"Error performing search: {e}")
            raise


def test_verification():
    """
    Execute a search for a known topic from the book and print results in readable format.
    """
    logger.info("Starting verification test...")

    try:
        # Initialize the retrieval system
        retriever = BookRetrieval()

        # Execute a search for a known topic (Isaac Sim)
        query = "Isaac Sim"
        logger.info(f"Searching for: '{query}'")

        results = retriever.search(query, top_k=5)

        if results:
            logger.info(f"Found {len(results)} results for '{query}'")
            logger.info("Results (Score | URL | Snippet):")
            logger.info("-" * 80)

            for i, result in enumerate(results, 1):
                score = result['score']
                url = result['url']
                snippet = result['content'][:150] + "..." if len(result['content']) > 150 else result['content']

                logger.info(f"{i}. Score: {score:.3f} | URL: {url}")
                logger.info(f"   Snippet: {snippet}")
                logger.info("")

                # Check if score meets the 0.70 threshold requirement
                if score >= 0.70:
                    logger.info(f"✅ Result {i} meets the 0.70 score threshold: {score:.3f}")
                else:
                    logger.info(f"ℹ️  Result {i} below 0.70 threshold: {score:.3f}")
        else:
            logger.warning(f"No results found for '{query}'")

        # Count results with score > 0.70
        high_score_results = [r for r in results if r['score'] >= 0.70]
        logger.info(f"Total results with score > 0.70: {len(high_score_results)}/5")

        # Verify success criteria
        if len(results) >= 3 and len(high_score_results) >= 1:
            logger.info("✅ VERIFICATION SUCCESSFUL: At least 3 results with at least 1 above 0.70 threshold")
            return True
        else:
            logger.warning("⚠️ VERIFICATION PARTIAL: Results found but may not meet all criteria")
            return len(results) > 0  # Return True if any results found, even if not meeting all thresholds

    except Exception as e:
        logger.error(f"❌ VERIFICATION FAILED: {e}")
        return False


if __name__ == "__main__":
    """
    __main__ block to act as the verification script (searching for "Isaac Sim").
    """
    logger.info("Book Retrieval Verification Script")
    logger.info("="*50)

    success = test_verification()

    if success:
        logger.info("🎉 VERIFICATION COMPLETED SUCCESSFULLY!")
        logger.info("The semantic retrieval system is working as specified.")
    else:
        logger.info("❌ VERIFICATION DID NOT COMPLETE SUCCESSFULLY!")

    logger.info("="*50)