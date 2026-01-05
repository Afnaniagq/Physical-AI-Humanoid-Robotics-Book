# Chatbot Integration

## Overview

This document describes the integration of the AI chatbot into the Physical AI & Humanoid Robotics book website. The chatbot allows users to ask questions about the book content and receive intelligent responses powered by an AI agent.

## Architecture

The chatbot integration consists of:

- **Backend**: FastAPI server with a `/chat` endpoint that interfaces with the AI agent
- **Frontend**: React component that provides the chat interface and communicates with the backend
- **Session Management**: Uses localStorage to maintain conversation context across page refreshes

## Backend API

### Endpoints

#### POST /chat

Sends a user query to the AI agent and returns a formatted response.

**Request Body:**
```json
{
  "message": "string (required) - The user's query or message",
  "session_id": "string (required) - Unique identifier for the conversation session"
}
```

**Response:**
```json
{
  "response": "string - The AI agent's response with Markdown formatting preserved",
  "session_id": "string - The session identifier from the request",
  "source_urls": "array of strings - List of source URLs referenced in the response",
  "timestamp": "string - ISO 8601 timestamp of when the response was generated"
}
```

### Error Handling

The API returns appropriate HTTP status codes:
- `200`: Success
- `400`: Bad request (validation error)
- `500`: Internal server error

## Frontend Component

The chatbot component is built with React and includes:

- Real-time messaging interface
- Markdown rendering for AI responses
- Session persistence using localStorage
- Loading indicators
- Error handling and display

## Environment Variables

The backend requires the following environment variables:

- `OPENROUTER_API_KEY`: API key for OpenRouter service
- `QDRANT_HOST`: Host URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant vector database

## Running the Application

### Backend

```bash
cd backend
pip install -r requirements.txt
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

### Frontend

```bash
cd frontend
npm install
npm start
```

The backend runs on `http://localhost:8000` and the frontend on `http://localhost:3000`. CORS is configured to allow communication between them.

## Troubleshooting

### Common Issues

1. **CORS Errors**: Ensure the backend is running and CORS is properly configured
2. **API Key Issues**: Verify that all required environment variables are set
3. **Connection Issues**: Check that the frontend is trying to connect to the correct backend URL

## Security Considerations

- API keys are loaded from environment variables and not exposed to the frontend
- Input validation is performed on the backend
- Rate limiting should be implemented in production