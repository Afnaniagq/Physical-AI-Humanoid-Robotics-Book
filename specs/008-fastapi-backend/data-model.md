# Data Model: FastAPI Backend for AI Agent Integration

## Entity: ChatRequest
**Description**: Represents a user's chat message and session information sent to the API

**Fields**:
- message: string (required) - The user's query or message
- session_id: string (required) - Unique identifier for the conversation session

**Validation Rules**:
- message must be between 1 and 10000 characters
- session_id must be a valid string identifier
- message cannot be empty or whitespace only

## Entity: ChatResponse
**Description**: Represents the AI agent's response to a user's query

**Fields**:
- response: string (required) - The AI agent's response with Markdown formatting preserved
- session_id: string (required) - The session identifier from the request
- source_urls: array of strings (optional) - List of source URLs referenced in the response
- timestamp: string (required) - ISO 8601 timestamp of when the response was generated

**Validation Rules**:
- response must be between 1 and 50000 characters
- source_urls must be valid URL strings if provided
- timestamp must be in ISO 8601 format

## Entity: ConversationSession
**Description**: Maintains the state and history of a user's conversation across multiple requests

**Fields**:
- session_id: string (required) - Unique identifier for the conversation
- history: array of objects (required) - Array of {role: string, content: string} objects representing the conversation history
- created_at: string (required) - ISO 8601 timestamp of when the session was created
- last_accessed: string (required) - ISO 8601 timestamp of the last interaction

**State Transitions**:
- Created: When a new session_id is provided that doesn't exist
- Updated: When a new message is added to the conversation history
- Maintained: When existing session_id is used for subsequent requests

## Entity: APIError
**Description**: Represents an error response from the API

**Fields**:
- error: string (required) - Descriptive error message
- status_code: number (required) - HTTP status code
- timestamp: string (required) - ISO 8601 timestamp of when the error occurred
- details: object (optional) - Additional error details for debugging

**Validation Rules**:
- error must be a non-empty string
- status_code must be a valid HTTP error code (400-599)
- timestamp must be in ISO 8601 format