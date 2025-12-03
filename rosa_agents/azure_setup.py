# azure_setup.py
from dotenv import load_dotenv
import os
from langchain_openai import AzureChatOpenAI

# Load .env file
load_dotenv()

# Read keys from environment
AZURE_API_KEY = os.getenv("AZURE_OPENAI_API_KEY")
AZURE_ENDPOINT = os.getenv("AZURE_OPENAI_ENDPOINT")
AZURE_DEPLOYMENT = os.getenv("AZURE_DEPLOYMENT_NAME")
OPENAI_VERSION = os.getenv("OPENAI_API_VERSION")

# Initialize Azure OpenAI LLM
llm = AzureChatOpenAI(
    api_key=AZURE_API_KEY,
    azure_endpoint=AZURE_ENDPOINT,
    deployment_name=AZURE_DEPLOYMENT,
    api_version=OPENAI_VERSION
)

print("✅ Azure OpenAI LLM initialized successfully.")
