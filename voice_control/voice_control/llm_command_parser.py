import json

def llm_to_ros_command(llm_json: str) -> dict:
    """
    Преобразует JSON-ответ LLM в JSON-команду для робота.
    Возвращает Python-словарь (готов для json.dumps() и отправки по сети).
    """
    try:
        data = json.loads(llm_json)
        response = data.get("response", "")
        parts = response.split(";")

        ros_cmd = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

        if parts[0] == "command" and parts[1] == "stop":
            ros_cmd["linear"]["x"] = 0.0
            ros_cmd["angular"]["z"] = 0.0

        elif parts[0] == "movement" and len(parts) == 3:
            direction = parts[1]
            speed = float(parts[2])

            if direction == "forward":
                ros_cmd["linear"]["x"] = speed
            elif direction == "backward":
                ros_cmd["linear"]["x"] = -speed

        else:
            ros_cmd["error"] = f"Unknown command: {response}"

        return ros_cmd

    except Exception as e:
        return {"error": str(e)}


# --- пример использования ---
if __name__ == "__main__":
    llm_move = json.dumps({
        "response": "movement;forward;1.2"
    })

    llm_stop = json.dumps({
        "response": "command;stop"
    })

    print("Move →", json.dumps(llm_to_ros_command(llm_move), indent=2, ensure_ascii=False))
    print("\nStop →", json.dumps(llm_to_ros_command(llm_stop), indent=2, ensure_ascii=False))
