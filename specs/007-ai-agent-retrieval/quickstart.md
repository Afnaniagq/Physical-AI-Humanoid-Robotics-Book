# Quickstart Guide: AI Agent with OpenAI Agents SDK & Retrieval Integration

## Prerequisites
- Python 3.11+
- OpenAI Agents SDK
- Access to OpenAI API (with gpt-4o-mini or gpt-4o model access)
- Existing backend/retrieve.py with BookRetrieval class
- Qdrant vector database with book content

## Setup
1. Install required dependencies:
   ```bash
   pip install openai agents
   ```

2. Ensure backend/retrieve.py exists with BookRetrieval class

3. Set up OpenAI API key in environment

## Usage
1. Run the agent directly:
   ```bash
   python backend/agent.py
   ```

2. The agent will initialize and run a test query "What is Isaac Sim?"

## Testing
- The agent includes a test block that runs a sample query
- Verify the agent responds with content from the book and proper citations
- Test out-of-scope queries to ensure proper error handling