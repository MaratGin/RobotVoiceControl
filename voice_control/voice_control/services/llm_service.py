import requests
from configs import Config

def send_to_llm(prompt):
    # Отправка запроса к LLM-серверу Ollama
    print("SEND TO LLM - " + prompt)
    response = requests.post(
        f"{Config.OLLAMA_SERVER}/api/generate",
        json={"model": "llama3.1:8b", "system": Config.SYSTEM_PROMPT, "prompt": prompt, "stream": False},
        timeout=60
    )
    print(2)
    llm_response = response.json()
    print(3)
    # Отправка ответа на ROS мост (оставлено как было) — не меняем формат отправки
    bridge_response = requests.post(
        Config.ROS_BRIDGE_URL,
        json=llm_response,
        timeout=10
    )
    print(4)
    print("LLM response:", llm_response)
    # Возвращаем исходный LLM-ответ и распарсенное поле done
    return llm_response
