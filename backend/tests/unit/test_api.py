import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from backend.api import app
from backend.agent import ConversationManager


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


def test_root_endpoint(client):
    """Test the root endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"message": "AI Agent Chat API is running!"}


@patch('backend.api.get_conversation_manager')
def test_chat_endpoint_success(mock_get_conversation_manager, client):
    """Test the chat endpoint with a successful response"""
    # Mock the conversation manager
    mock_conversation_manager = MagicMock()
    mock_conversation_manager.chat.return_value = "This is a test response"
    mock_get_conversation_manager.return_value = mock_conversation_manager

    # Test data
    test_data = {
        "message": "Hello, how are you?",
        "session_id": "test_session_123"
    }

    response = client.post("/chat", json=test_data)

    # Assertions
    assert response.status_code == 200
    assert "response" in response.json()
    assert "session_id" in response.json()
    assert response.json()["session_id"] == "test_session_123"


def test_chat_endpoint_validation_error(client):
    """Test the chat endpoint with invalid input"""
    # Test with empty message
    test_data = {
        "message": "",  # Empty message should fail validation
        "session_id": "test_session_123"
    }

    response = client.post("/chat", json=test_data)

    # Should return 422 for validation error
    assert response.status_code == 422


def test_chat_endpoint_missing_fields(client):
    """Test the chat endpoint with missing required fields"""
    # Test with missing message
    test_data = {
        "session_id": "test_session_123"
    }

    response = client.post("/chat", json=test_data)

    # Should return 422 for validation error
    assert response.status_code == 422


@patch('backend.api.get_conversation_manager')
def test_chat_endpoint_agent_error(mock_get_conversation_manager, client):
    """Test the chat endpoint when agent raises an exception"""
    # Mock the conversation manager to raise an exception
    mock_conversation_manager = MagicMock()
    mock_conversation_manager.chat.side_effect = Exception("Agent error")
    mock_get_conversation_manager.return_value = mock_conversation_manager

    # Test data
    test_data = {
        "message": "Hello",
        "session_id": "test_session_123"
    }

    response = client.post("/chat", json=test_data)

    # Should return 500 for internal server error
    assert response.status_code == 500