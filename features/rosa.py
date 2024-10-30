from langchain_ollama import ChatOllama
from rosa import ROSA

ollama_llm = ChatOllama(
    model="llama3.1:70b",  # or your preferred model
    temperature=0,
    num_ctx=8192,  # adjust based on your model's context window
)

# Pass the LLM to ROSA
rosa_instance = ROSA(ros_version=2, llm=ollama_llm)
rosa_instance.invoke("Show me a list of topics that have publishers but no subscribers")  