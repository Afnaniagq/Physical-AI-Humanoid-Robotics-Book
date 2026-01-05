"""
AI Agent with OpenAI Agents SDK & Retrieval Integration

Implements a conversational AI agent using the OpenAI Agents SDK that integrates
with the BookRetrieval class to answer questions about the "Physical AI & Humanoid Robotics" book.
"""

from agents import Agent, Runner, function_tool
import os
from dotenv import load_dotenv
from typing import Dict, List, Any
from agents import OpenAIChatCompletionsModel
from openai import AsyncOpenAI
import asyncio

# Import the BookRetrieval class
from .retrieve import BookRetrieval


# Load environment variables
load_dotenv()

client = AsyncOpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1"
)

third_party_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="mistralai/devstral-2512:free"
)

@function_tool
def search_book_content(query: str) -> str:
    """
    Search the book content using the BookRetrieval class and return formatted results.

    Args:
        query (str): The search query string

    Returns:
        str: Formatted string containing retrieved text and associated URLs
    """
    try:
        # Instantiate the BookRetrieval class
        retriever = BookRetrieval()

        # Perform the search
        results = retriever.search(query, top_k=5)

        if not results:
            return "No relevant results found in the book for your query."

        # Format the results with Markdown formatting and source citations
        formatted_results = []
        for result in results:
            content = result.get('content', '')[:500]  # Limit content length
            url = result.get('url', '')
            score = result.get('score', 0)

            # Format as Markdown with proper structure
            formatted_result = f"### Content:\n{content}...\n\n**Source:** [{url}]({url})\n\n*Relevance Score:* {score:.3f}\n\n---\n"
            formatted_results.append(formatted_result)

        return "## Found the following information:\n\n" + "\n".join(formatted_results)

    except Exception as e:
        return f"Error searching book content: {str(e)}"

# Create the AI agent with enhanced instructions for context awareness
agent = Agent(
    name="Humanoid Robotics Expert",
    # model="gpt-4o-mini",  # Using gpt-4o-mini for efficiency as specified
    instructions="""You are a professional robotics expert. Use the search_book_content tool to find answers.
    Maintain context from previous interactions in the conversation.
    If the information is not in the book, say: "I'm sorry, that information is not in the book."
    Always provide the URL source link at the end of your answer.
    When responding to follow-up questions, reference the context from previous exchanges.""",
    model=third_party_model,
    tools=[search_book_content]
)

class ConversationManager:
    """
    Manages conversation context and history for the AI agent.
    """
    def __init__(self):
        self.conversation_history = []

    def add_message(self, role: str, content: str):
        """Add a message to the conversation history."""
        self.conversation_history.append({
            "role": role,
            "content": content
        })

    def get_context(self) -> str:
        """Get the conversation context as a string."""
        if not self.conversation_history:
            return "No previous conversation history."

        context = "Previous conversation history:\n"
        for msg in self.conversation_history[-5:]:  # Include last 5 messages
            context += f"{msg['role']}: {msg['content']}\n"
        return context

    async def chat(self, user_input: str):
        """Process a user input with conversation context."""
        # Get the conversation context
        context = self.get_context()

        # Create a full prompt that combines context with new user input
        full_prompt = f"{context}\n\nCurrent user query: {user_input}"

        # Run the agent with the full prompt containing context
        result = await Runner.run(agent, full_prompt)

        # Add user message to history after getting result
        self.add_message("user", user_input)

        # Add agent response to history
        self.add_message("assistant", str(result))

        return result.final_output

if __name__ == "__main__":
    

    async def main():
        # Everything below this line MUST be indented!
        conversation = ConversationManager()

        # Test the agent with a sample query
        result = await conversation.chat("What is Isaac Sim?")
        print("Agent Response:")
        print(result)

        # Test follow-up question
        follow_up_result = await conversation.chat("Can you tell me more about its features?")
        print("\nFollow-up Agent Response:")
        print(follow_up_result)

    # This line must be outside the main function, 
    # lined up with "async def"
    asyncio.run(main())