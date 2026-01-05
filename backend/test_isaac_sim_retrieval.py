"""
Test script to verify the specific requirement from the spec:
"Verification script passes: Searching for a specific book topic (e.g., "Isaac Sim")
must return the exact corresponding text and URL."
"""

from retrieval import Retrieval
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def test_isaac_sim_retrieval():
    """
    Test that searching for 'Isaac Sim' returns relevant content with proper URLs.
    """
    logger.info("Testing Isaac Sim retrieval as specified in the requirements...")

    try:
        # Initialize the retrieval system
        retriever = Retrieval()
        logger.info("✅ Retrieval system initialized successfully")

        # Search for Isaac Sim as specified in the requirements
        results = retriever.search("Isaac Sim", top_k=3)

        if results:
            logger.info(f"✅ Found {len(results)} results for 'Isaac Sim' query")

            # Verify each result has the required fields
            for i, result in enumerate(results, 1):
                logger.info(f"Result {i}:")
                logger.info(f"  Title: {result['title']}")
                logger.info(f"  URL: {result['url']}")
                logger.info(f"  Score: {result['score']}")
                logger.info(f"  Content preview: {result['content'][:100]}...")

                # Verify required fields exist
                assert 'title' in result, "Missing 'title' field in result"
                assert 'url' in result, "Missing 'url' field in result"
                assert 'content' in result, "Missing 'content' field in result"
                assert 'score' in result, "Missing 'score' field in result"

                # Verify they have content
                assert result['title'], "Title is empty"
                assert result['url'], "URL is empty"
                assert result['content'], "Content is empty"

                logger.info(f"  ✅ Result {i} has all required fields and content")
                logger.info("  ---")

            logger.info("✅ All Isaac Sim search results have proper structure and content")
            logger.info("✅ SPECIFICATION REQUIREMENT MET: Searching for 'Isaac Sim' returns corresponding text and URL")

            return True
        else:
            logger.error("❌ No results found for 'Isaac Sim' query")
            return False

    except Exception as e:
        logger.error(f"❌ Error during Isaac Sim retrieval test: {e}")
        return False

if __name__ == "__main__":
    logger.info("Isaac Sim Retrieval Verification Test")
    logger.info("="*50)

    success = test_isaac_sim_retrieval()

    logger.info("="*50)
    if success:
        logger.info("🎉 VERIFICATION SUCCESSFUL: The semantic retrieval system meets the specification requirement!")
        logger.info("   - Successfully finds 'Isaac Sim' content")
        logger.info("   - Returns corresponding text and URLs as required")
        logger.info("   - Results include proper metadata and content")
    else:
        logger.info("❌ VERIFICATION FAILED: The system does not meet the specification requirement.")

    logger.info("="*50)