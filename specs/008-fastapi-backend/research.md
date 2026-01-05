# Research Notes: FastAPI Backend for AI Agent Integration

## Decision: FastAPI Application Structure
**Rationale**: Using a single file (backend/api.py) for the FastAPI application based on the user's specific requirement and the simplicity of the API. This follows FastAPI's simple application pattern for smaller services.

**Alternatives considered**:
- Multi-file structure with separate routes, models, and services: More complex than needed for a single endpoint
- Using existing backend structure: User specifically requested backend/api.py

## Decision: CORS Configuration
**Rationale**: Using CORSMiddleware specifically allowing http://localhost:3000 as specified in the user requirements to enable communication between Docusaurus frontend and FastAPI backend during development.

**Alternatives considered**:
- Allow all origins: Security risk
- Environment-based configuration: More complex but better for production

## Decision: Frontend Component Structure
**Rationale**: Creating a Chatbot component with index.tsx and CSS modules to provide a self-contained, styled component that integrates well with Docusaurus and supports dark mode.

**Alternatives considered**:
- Single file component: Less organized
- Different styling approach: CSS modules provide better encapsulation

## Decision: Session Management
**Rationale**: Using localStorage for session persistence as specified in user requirements to maintain conversation history across page refreshes.

**Alternatives considered**:
- Server-side session storage: More complex but more secure
- URL parameters: Would clutter the URL

## Decision: Markdown Rendering
**Rationale**: Using react-markdown as specified in user requirements to properly render the agent's responses with links and formatting.

**Alternatives considered**:
- dangerouslySetInnerHTML: Security risk
- Custom parsing: More complex than needed

## Backend Dependencies to Install
- fastapi: For the web framework
- uvicorn: ASGI server
- python-multipart: For handling form data if needed
- python-dotenv: For environment variable loading

## Frontend Dependencies to Install
- react-markdown: For rendering markdown content
- remark-gfm: For GitHub Flavored Markdown support
- @types/react-markdown: TypeScript definitions

## Integration Points
- backend/agent.py: Import ConversationManager and agent classes
- Environment loading: Use existing .env file configuration
- Docusaurus integration: Add component to root layout