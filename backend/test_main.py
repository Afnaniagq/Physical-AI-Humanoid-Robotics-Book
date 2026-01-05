import unittest
from unittest.mock import Mock, patch
import os
from main import DataIngestionPipeline


class TestDataIngestionPipeline(unittest.TestCase):

    def setUp(self):
        # Mock environment variables to avoid actual API calls
        with patch.dict(os.environ, {
            'COHERE_API_KEY': 'test_key',
            'QDRANT_URL': 'http://test-url',
            'QDRANT_API_KEY': 'test_key'
        }):
            with patch('qdrant_client.QdrantClient') as mock_qdrant:
                mock_qdrant.return_value.get_collections.return_value = Mock()
                mock_qdrant.return_value.get_collections.return_value.collections = []
                self.pipeline = DataIngestionPipeline()

    def test_chunk_text_short_text(self):
        """Test that short text is not chunked"""
        text = "This is a short text."
        chunks = self.pipeline.chunk_text(text, max_chunk_size=100)
        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0], text)

    def test_chunk_text_long_text(self):
        """Test that long text is properly chunked"""
        long_text = "This is a sentence. " * 50  # Creates a long text
        chunks = self.pipeline.chunk_text(long_text, max_chunk_size=100)
        # Should be split into multiple chunks
        self.assertGreater(len(chunks), 1)
        # Each chunk should be less than or equal to max_chunk_size
        for chunk in chunks:
            self.assertLessEqual(len(chunk), 100)


if __name__ == '__main__':
    unittest.main()