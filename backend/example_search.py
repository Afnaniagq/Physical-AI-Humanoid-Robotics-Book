"""
Example usage script for the Data Ingestion Pipeline.

This script demonstrates how to use the pipeline to process a single URL
and shows how to validate the setup before running the full pipeline.
"""

import os
import sys
from main import DataIngestionPipeline
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def example_usage():
    """Example of how to use the DataIngestionPipeline class."""
    logger.info("Starting example usage of Data Ingestion Pipeline")

    # Check for required environment variables
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        logger.error("Please set these variables in your .env file")
        return False

    # Initialize the pipeline
    logger.info("Initializing Data Ingestion Pipeline...")
    pipeline = DataIngestionPipeline()

    # Validate clients before starting
    logger.info("Validating API clients...")
    if not pipeline.validate_clients():
        logger.error("Client validation failed. Please check your API keys and network connection.")
        return False

    # Example: Process a single URL for demonstration
    test_url = "https://physical-ai-humanoid-robotics-book-bay.vercel.app/docs/intro"  # Example Docusaurus URL

    logger.info(f"Processing example URL: {test_url}")

    # Extract content
    content_data = pipeline.extract_content_from_url(test_url)
    if content_data:
        logger.info(f"Successfully extracted content: {len(content_data['content'])} characters from '{content_data['title']}'")

        # Chunk the content
        chunks = pipeline.chunk_text(content_data['content'])
        logger.info(f"Content chunked into {len(chunks)} pieces")

        # Show first chunk as example
        if chunks:
            logger.info(f"First chunk preview: {chunks[0][:100]}...")

        # Verify chunk quality
        quality_metrics = pipeline.verify_chunk_quality(content_data['content'], chunks)
        logger.info(f"Chunk quality metrics: {quality_metrics}")

        logger.info("Example usage completed successfully!")
        return True
    else:
        logger.error(f"Failed to extract content from {test_url}")
        return False

def run_verification_test():
    """Run a verification test to ensure the pipeline works end-to-end."""
    logger.info("Running verification test...")

    # Check for required environment variables
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        return False

    # Initialize the pipeline
    pipeline = DataIngestionPipeline()

    # Validate clients
    if not pipeline.validate_clients():
        logger.error("Client validation failed")
        return False

    # Test with a simple operation
    try:
        # Test chunking with sample text
        sample_text = "This is a sample text for testing the chunking functionality. " * 20  # Make it long enough to chunk
        chunks = pipeline.chunk_text(sample_text)
        logger.info(f"Successfully chunked sample text into {len(chunks)} chunks")

        # Test chunk quality verification
        quality_metrics = pipeline.verify_chunk_quality(sample_text, chunks)
        logger.info(f"Sample text chunking quality: {quality_metrics}")

        logger.info("Verification test passed!")
        return True
    except Exception as e:
        logger.error(f"Verification test failed: {e}")
        return False

if __name__ == "__main__":
    logger.info("Example Usage Script for Data Ingestion Pipeline")
    logger.info("="*50)

    # Run verification test first
    logger.info("Step 1: Running verification test...")
    verification_success = run_verification_test()

    if verification_success:
        logger.info("\nStep 2: Running example usage...")
        example_success = example_usage()

        if example_success:
            logger.info("\nBoth verification and example usage completed successfully!")
            logger.info("The pipeline is properly configured and ready to use.")
        else:
            logger.error("\nExample usage failed, but verification passed.")
    else:
        logger.error("Verification test failed. Please check your configuration.")

    logger.info("="*50)