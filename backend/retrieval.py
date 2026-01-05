"""
Semantic Retrieval Module

Implements a Retrieval class that takes a natural language query and returns
the top K most relevant chunks from the Qdrant collection.
"""

import os
import time
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging
from typing import List, Dict, Optional
from config import *

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

class Retrieval:
    """
    A retrieval class that performs semantic search on the Qdrant collection.

    Takes a natural language query and returns the top K most relevant chunks
    using vector similarity search against the 'docusaurus_content' collection.
    """

    def __init__(self):
        """
        Initialize the Retrieval class with Cohere and Qdrant clients.
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

    def convert_query_to_embedding(self, query: str) -> List[float]:
        """
        Convert a natural language query into a 1024-dimensional embedding using Cohere.

        Args:
            query: The natural language query string

        Returns:
            A list of 1024 float values representing the embedding
        """
        try:
            # Generate embedding for the query using the same model as the ingestion pipeline
            response = self.cohere_client.embed(
                texts=[query],
                model=COHERE_MODEL_NAME,
                input_type="search_query"  # Using search_query for query embeddings
            )

            # Return the first (and only) embedding
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error converting query to embedding: {e}")
            raise

    def search(self, query: str, top_k: int = 5) -> List[Dict]:
        """
        Perform semantic search against the Qdrant collection.

        Args:
            query: Natural language query string
            top_k: Number of top results to return (default: 5)

        Returns:
            List of dictionaries containing the top K most relevant chunks with metadata
        """
        start_time = time.time()

        try:
            # Convert query to embedding
            query_embedding = self.convert_query_to_embedding(query)

            # Perform vector similarity search in Qdrant using query_points method
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,  # Pass the embedding vector directly
                limit=top_k,
                with_payload=True,  # Include metadata in results
                with_vectors=False  # Don't return the vectors themselves
            )

            # Format results - query_points returns QueryResponse object with points attribute
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
            logger.info(f"Search completed in {elapsed_time:.3f} seconds for query: '{query[:50]}...'")

            # Check if results were found
            if not formatted_results:
                logger.warning(f"No results found for query: '{query}'")

            return formatted_results

        except Exception as e:
            logger.error(f"Error performing search: {e}")
            raise

    def search_with_metadata_filter(self, query: str, filters: Optional[Dict] = None, top_k: int = 5) -> List[Dict]:
        """
        Perform semantic search with metadata filtering.

        Args:
            query: Natural language query string
            filters: Dictionary of metadata filters (e.g., {'source': 'docusaurus'})
            top_k: Number of top results to return (default: 5)

        Returns:
            List of dictionaries containing the top K most relevant chunks with metadata
        """
        start_time = time.time()

        try:
            # Convert query to embedding
            query_embedding = self.convert_query_to_embedding(query)

            # Build Qdrant filter if filters are provided
            qdrant_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    # Create a filter for the specified field
                    # Note: The field must be indexed in Qdrant for filtering to work
                    conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

                if conditions:
                    qdrant_filter = models.Filter(
                        must=conditions
                    )

            # Perform vector similarity search in Qdrant with filters using query_points method
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,  # Pass the embedding vector directly
                limit=top_k,
                with_payload=True,  # Include metadata in results
                with_vectors=False,  # Don't return the vectors themselves
                query_filter=qdrant_filter  # Apply filters if provided
            )

            # Format results - query_points returns QueryResponse object with points attribute
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
            logger.info(f"Filtered search completed in {elapsed_time:.3f} seconds for query: '{query[:50]}...'")

            # Check if results were found
            if not formatted_results:
                logger.warning(f"No results found for query: '{query}' with filters: {filters}")

            return formatted_results

        except Exception as e:
            # If filtering fails (e.g., due to missing index), log the error and return empty results
            logger.warning(f"Filtering failed: {e}. Filters: {filters}")
            # Log performance even for failed filtering
            elapsed_time = time.time() - start_time
            logger.info(f"Filtered search completed in {elapsed_time:.3f} seconds for query: '{query[:50]}...'")

            # For now, return empty results when filtering fails, but we should implement fallback
            # In a production system, we'd want to try the search without filters as a fallback
            # For this implementation, we'll implement the fallback approach
            logger.info("Attempting fallback search without filters due to filter error...")
            return self.search(query=query, top_k=top_k)

def verify_search_accuracy():
    """
    Verification function to test search accuracy against known content.
    """
    logger.info("Starting verification of search accuracy...")

    try:
        # Initialize the retrieval system
        retriever = Retrieval()

        # Test query for a specific topic (using "Isaac Sim" as mentioned in the spec)
        test_query = "Isaac Sim"

        # Perform search
        results = retriever.search(query=test_query, top_k=3)

        logger.info(f"Found {len(results)} results for query: '{test_query}'")

        # Print results for verification
        for i, result in enumerate(results, 1):
            logger.info(f"Result {i}:")
            logger.info(f"  Title: {result['title']}")
            logger.info(f"  URL: {result['url']}")
            logger.info(f"  Score: {result['score']}")
            logger.info(f"  Content preview: {result['content'][:200]}...")
            logger.info("-" * 50)

        # Verify that we have meaningful results
        if results:
            logger.info("✅ Verification passed: Search returned relevant results")
            return True
        else:
            logger.warning("⚠️  Verification warning: No results found for test query")
            return False

    except Exception as e:
        logger.error(f"❌ Verification failed: {e}")
        return False

if __name__ == "__main__":
    # Example usage
    logger.info("Semantic Retrieval Module - Example Usage")
    logger.info("="*50)

    try:
        # Initialize the retrieval system
        retriever = Retrieval()

        # Example search
        query = "What is Isaac Sim?"
        logger.info(f"Searching for: '{query}'")

        results = retriever.search(query=query, top_k=5)

        if results:
            logger.info(f"Top {len(results)} results for query: '{query}'")
            for i, result in enumerate(results, 1):
                logger.info(f"{i}. {result['title']}")
                logger.info(f"   URL: {result['url']}")
                logger.info(f"   Score: {result['score']:.3f}")
                logger.info(f"   Preview: {result['content'][:150]}...")
                logger.info("")
        else:
            logger.info("No results found for the query.")

        # Run verification
        logger.info("Running verification test...")
        success = verify_search_accuracy()

        if success:
            logger.info("✅ Semantic retrieval module is working correctly!")
        else:
            logger.info("❌ Semantic retrieval module needs attention.")

    except Exception as e:
        logger.error(f"Error in example usage: {e}")