import os
import dotenv
from langchain_openai import ChatOpenAI


from rich.console import Console


def get_env_variable(name: str) -> str:
    """Get environment variable with error handling."""
    value = os.getenv(name)
    if value is None:
        raise ValueError(f"Environment variable {name} is not set")
    return value


def get_llm(streaming: bool = False):
    """Get the LLM instance for drone control."""
    dotenv.load_dotenv(dotenv.find_dotenv())
    
    # Try to get OpenAI API key
    api_key = os.getenv("OPENAI_API_KEY")
    if api_key:
        llm = ChatOpenAI(
            model="gpt-4",
            temperature=0.1,
            streaming=streaming,
            api_key=api_key
        )
        return llm
    
    # Fallback to local model if available
    try:
        from langchain_ollama import ChatOllama
        llm = ChatOllama(
            model="llama3.1:latest",
            temperature=0.1,
            num_ctx=8192,
        )
        return llm
    except ImportError:
        pass
    
    # If no LLM available, raise error
    raise ValueError(
        "No LLM configuration found. Please set OPENAI_API_KEY environment variable "
        "or install ollama with langchain_ollama package."
    )