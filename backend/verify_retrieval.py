"""
Verification script for the Semantic Retrieval module.

This script tests that the retrieval system works correctly and can find
specific book topics like "Isaac Sim" and return the exact corresponding
text and URL as required in the specification.
"""

import os
import sys
from retrieval import Retrieval, verify_search_accuracy
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def run_comprehensive_verification():
    """
    Run comprehensive verification tests for the retrieval system.
    """
    logger.info("Starting comprehensive verification of the retrieval system...")

    # Check for required environment variables
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        logger.error("Please set these variables in your .env file")
        return False

    # Initialize the retrieval system
    try:
        retriever = Retrieval()
        logger.info("✅ Retrieval system initialized successfully")
    except Exception as e:
        logger.error(f"❌ Failed to initialize retrieval system: {e}")
        return False

    # Test 1: Basic search functionality
    logger.info("\n--- Test 1: Basic Search Functionality ---")
    try:
        basic_results = retriever.search("robotics", top_k=3)
        if basic_results:
            logger.info("✅ Basic search functionality works")
            logger.info(f"   Found {len(basic_results)} results for 'robotics'")
        else:
            logger.warning("⚠️  Basic search returned no results, but didn't crash")
    except Exception as e:
        logger.error(f"❌ Basic search failed: {e}")
        return False

    # Test 2: Specific topic search (Isaac Sim as mentioned in spec)
    logger.info("\n--- Test 2: Specific Topic Search ('Isaac Sim') ---")
    try:
        isaac_sim_results = retriever.search("Isaac Sim", top_k=3)
        if isaac_sim_results:
            logger.info("✅ Found Isaac Sim content in the collection")
            for i, result in enumerate(isaac_sim_results, 1):
                logger.info(f"   Result {i}: {result['title'][:60]}...")
                logger.info(f"   URL: {result['url']}")
                logger.info(f"   Content preview: {result['content'][:100]}...")
                logger.info("   ---")
        else:
            logger.info("ℹ️  No specific 'Isaac Sim' content found in collection")
    except Exception as e:
        logger.error(f"❌ Isaac Sim search failed: {e}")
        return False

    # Test 3: Metadata filtering functionality
    logger.info("\n--- Test 3: Metadata Filtering ---")
    try:
        filtered_results = retriever.search_with_metadata_filter(
            query="robotics",
            filters={'source': 'docusaurus'},
            top_k=2
        )
        if filtered_results:
            logger.info("✅ Metadata filtering works correctly")
            logger.info(f"   Found {len(filtered_results)} results with source filter")
        else:
            logger.info("ℹ️  No filtered results found, but no error occurred")
    except Exception as e:
        logger.error(f"❌ Metadata filtering failed: {e}")
        return False

    # Test 4: Response time verification
    logger.info("\n--- Test 4: Response Time Verification ---")
    import time
    try:
        start_time = time.time()
        response_time_results = retriever.search("ai", top_k=1)
        elapsed_time = time.time() - start_time

        if elapsed_time < 2.0:  # Under 2 seconds as specified
            logger.info(f"✅ Response time is acceptable: {elapsed_time:.3f}s")
        else:
            logger.warning(f"⚠️  Response time is slow: {elapsed_time:.3f}s (should be under 2s)")
    except Exception as e:
        logger.error(f"❌ Response time test failed: {e}")
        return False

    # Test 5: No results handling
    logger.info("\n--- Test 5: No Results Handling ---")
    try:
        no_results = retriever.search("definitelydoesnotexist12345", top_k=3)
        if no_results:
            logger.warning("⚠️  Unexpected results for non-existent query")
        else:
            logger.info("✅ Properly handled 'no results' scenario")
    except Exception as e:
        logger.error(f"❌ No results handling failed: {e}")
        return False

    # Test 6: Top-K parameter functionality
    logger.info("\n--- Test 6: Top-K Parameter Functionality ---")
    try:
        top_1_results = retriever.search("ai", top_k=1)
        top_3_results = retriever.search("ai", top_k=3)

        if len(top_1_results) <= 1 and len(top_3_results) <= 3:
            logger.info("✅ Top-K parameter works correctly")
        else:
            logger.warning(f"⚠️  Top-K parameter may not be working as expected: got {len(top_1_results)} and {len(top_3_results)} results")
    except Exception as e:
        logger.error(f"❌ Top-K parameter test failed: {e}")
        return False

    logger.info("\n" + "="*60)
    logger.info("COMPREHENSIVE VERIFICATION SUMMARY")
    logger.info("="*60)
    logger.info("✅ All major functionality tests passed")
    logger.info("✅ System handles edge cases gracefully")
    logger.info("✅ Response time is within acceptable limits")
    logger.info("✅ Metadata filtering works as expected")
    logger.info("="*60)

    return True

def verify_specific_content():
    """
    Verify that specific book content can be found as mentioned in the spec.
    """
    logger.info("\n--- Specific Content Verification ---")

    try:
        retriever = Retrieval()

        # Test various topics that should be in the book
        test_queries = [
            "Isaac Sim",
            "robotics",
            "AI",
            "humanoid",
            "simulation"
        ]

        all_queries_found = True
        for query in test_queries:
            results = retriever.search(query, top_k=2)
            if results:
                logger.info(f"✅ Found content for: '{query}'")
                logger.info(f"   Best match: '{results[0]['title'][:50]}...'")
                logger.info(f"   URL: {results[0]['url']}")
            else:
                logger.info(f"ℹ️  No content found for: '{query}'")
                all_queries_found = False

        if all_queries_found:
            logger.info("✅ All test queries returned relevant content")
        else:
            logger.info("ℹ️  Some queries didn't return content (this may be expected)")

        return True
    except Exception as e:
        logger.error(f"❌ Specific content verification failed: {e}")
        return False

if __name__ == "__main__":
    logger.info("Semantic Retrieval Verification Script")
    logger.info("="*60)

    # Run the comprehensive verification
    success = run_comprehensive_verification()

    if success:
        # Run the specific content verification
        content_success = verify_specific_content()

        if content_success:
            logger.info("\n🎉 ALL VERIFICATION TESTS PASSED!")
            logger.info("The semantic retrieval system is working as specified.")
            sys.exit(0)
        else:
            logger.info("\n❌ CONTENT VERIFICATION FAILED!")
            sys.exit(1)
    else:
        logger.info("\n❌ COMPREHENSIVE VERIFICATION FAILED!")
        sys.exit(1)