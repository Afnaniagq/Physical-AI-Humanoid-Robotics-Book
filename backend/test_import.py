"""
Test script to verify that backend/agent.py can be imported and used
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from agent import agent, ConversationManager

    print("SUCCESS: Successfully imported agent and ConversationManager from backend/agent.py")

    # Test that ConversationManager can be instantiated
    conversation_manager = ConversationManager()
    print("SUCCESS: Successfully instantiated ConversationManager")

    # Test that agent object exists
    print(f"SUCCESS: Agent object exists: {type(agent).__name__}")

    print("All imports successful!")

except ImportError as e:
    print(f"ERROR: Import error: {e}")
except Exception as e:
    print(f"ERROR: Error: {e}")