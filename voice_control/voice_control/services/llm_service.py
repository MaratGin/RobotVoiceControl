import requests
from configs import Config

def send_to_llm(prompt):
    # Отправка запроса к LLM-серверу Ollama
    response = requests.post(
        f"{Config.OLLAMA_SERVER}/api/generate",
        json={"model": "llama3.1:8b", "prompt": prompt},
        timeout=60
    )
    return response.json()
