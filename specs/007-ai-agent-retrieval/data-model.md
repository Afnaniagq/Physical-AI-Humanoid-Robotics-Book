# Data Model: AI Agent with OpenAI Agents SDK & Retrieval Integration

## Entity: AI Agent
- **Name**: "Humanoid Robotics Expert"
- **Model**: gpt-4o-mini (or gpt-4o)
- **Instructions**: Professional robotics expert that uses search_book_content tool to find answers
- **Behavior**: Responds with book content, provides source citations, handles out-of-scope queries gracefully

## Entity: Book Retrieval Tool
- **Function Name**: search_book_content(query: str)
- **Input**: Query string from the AI agent
- **Output**: Formatted string containing retrieved text and associated URLs
- **Integration**: Uses BookRetrieval class from backend/retrieve.py

## Entity: BookRetrieval Class
- **Source**: backend/retrieve.py
- **Method**: search() - performs vector search against Qdrant database
- **Return**: Retrieved text snippets and associated URLs

## Entity: Agent Response
- **Format**: Markdown
- **Content**: Answer based on retrieved text snippets
- **Citations**: URLs from the retrieved content
- **Error Handling**: Graceful response for out-of-scope queries