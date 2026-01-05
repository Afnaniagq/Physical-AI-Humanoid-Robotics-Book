# Research Document: AI Agent with OpenAI Agents SDK & Retrieval Integration

## Decision: OpenAI Agents SDK Implementation
**Rationale**: The user specifically requested using the OpenAI Agents SDK for this implementation. This SDK provides a clean interface for creating AI agents that can use tools, which is exactly what's needed to integrate with the BookRetrieval class.

## Decision: Tool Integration Pattern
**Rationale**: Using the @function_tool decorator pattern allows the agent to call the BookRetrieval.search() method as a tool. This is the standard approach for creating custom tools in the OpenAI Agents SDK.

## Decision: Model Selection (gpt-4o-mini vs gpt-4o)
**Rationale**: The user specified that either gpt-4o-mini (for efficiency) or gpt-4o can be used. gpt-4o-mini is recommended for cost and speed efficiency while still providing high-quality responses for this use case.

## Decision: Source Citation Format
**Rationale**: The retrieved URLs from Qdrant will be formatted and appended to the agent's response to provide proper source citations as required by the specification.

## Decision: Error Handling for Out-of-Scope Queries
**Rationale**: The agent will be configured with instructions to respond with "I'm sorry, that information is not in the book" when the BookRetrieval tool returns no relevant results.

## Decision: Agent Name and Persona
**Rationale**: The agent will be named "Humanoid Robotics Expert" to align with the book's subject matter and provide an appropriate persona for users asking questions about robotics content.