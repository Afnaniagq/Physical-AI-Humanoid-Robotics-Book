# Quickstart Guide: FastAPI Backend for AI Agent Integration

## Prerequisites
- Python 3.11+
- Node.js 18+ (for frontend development)
- Access to OpenRouter API (for AI agent)
- Qdrant Cloud access (for vector database)

## Backend Setup

### 1. Install Python Dependencies
```bash
cd backend
pip install fastapi uvicorn python-dotenv
```

### 2. Environment Configuration
Ensure your `.env` file at the project root contains:
```env
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_HOST=your_qdrant_host
QDRANT_API_KEY=your_qdrant_api_key
```

### 3. Start the Backend Server
```bash
cd backend
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

## Frontend Setup

### 1. Install Frontend Dependencies
```bash
cd frontend
npm install react-markdown remark-gfm
```

### 2. Add Chatbot Component to Docusaurus
Update your `docusaurus.config.js` to include the Chatbot component globally.

### 3. Start Docusaurus Development Server
```bash
cd frontend
npm run start
```

The Docusaurus site will be available at `http://localhost:3000`

## Testing the Integration

### 1. Verify Backend API
Test the chat endpoint directly:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is embodied intelligence?",
    "session_id": "test_session_123"
  }'
```

### 2. Test Frontend Integration
- Visit your Docusaurus site at `http://localhost:3000`
- Use the chatbot interface to ask questions about the book content
- Verify that responses preserve Markdown formatting
- Check browser console for any CORS errors

### 3. Validation Checklist
Run through these validation steps to ensure all components work together:

- [ ] Backend server starts without errors on `http://localhost:8000`
- [ ] Frontend Docusaurus site starts without errors on `http://localhost:3000`
- [ ] Chat endpoint accepts requests and returns responses
- [ ] CORS headers allow communication between frontend and backend
- [ ] Chatbot component appears on all pages of the Docusaurus site
- [ ] Messages sent from the frontend reach the backend
- [ ] Responses from the AI agent are properly formatted with Markdown
- [ ] Session persistence works across page refreshes
- [ ] Error handling displays properly in the UI
- [ ] Dark mode styling works correctly
- [ ] Unit tests pass: `cd backend && python -m pytest tests/unit/test_api.py`

## API Usage Example
The POST /chat endpoint expects:
```json
{
  "message": "Your question here",
  "session_id": "unique_session_identifier"
}
```

And returns:
```json
{
  "response": "Formatted response with **Markdown** support",
  "session_id": "same_session_identifier",
  "source_urls": ["https://reference-url.com"],
  "timestamp": "2025-12-28T10:30:00Z"
}
```

## Troubleshooting

### CORS Issues
If you see CORS errors in the browser console:
- Verify that the backend allows requests from `http://localhost:3000`
- Check that the CORSMiddleware is properly configured in backend/api.py

### Environment Variables
If you get authentication errors:
- Verify that your `.env` file contains the required API keys
- Ensure the backend is loading the `.env` file correctly

### Session Persistence
If conversation history isn't maintained across page refreshes:
- Check that localStorage is being used correctly in the frontend component
- Verify that the same session_id is being passed in subsequent requests