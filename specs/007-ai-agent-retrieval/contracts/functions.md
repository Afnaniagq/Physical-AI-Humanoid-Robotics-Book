# Function Contracts: AI Agent with OpenAI Agents SDK & Retrieval Integration

## Function: search_book_content(query: str)
- **Input**: Query string from the AI agent
- **Output**: Formatted string containing retrieved text and associated URLs
- **Behavior**: Calls BookRetrieval.search() method to retrieve relevant content from the book
- **Error Handling**: Returns appropriate message when no relevant results are found

## Function: Agent.run(query: str)
- **Input**: User query string
- **Output**: Formatted response with book content and source citations
- **Behavior**: Uses search_book_content tool to retrieve information, formats response in Markdown
- **Error Handling**: Responds with "I'm sorry, that information is not in the book" for out-of-scope queries