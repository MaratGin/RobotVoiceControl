from flask import Blueprint, request, jsonify
from services.llm_service import send_to_llm

llm_bp = Blueprint('llm', __name__)

@llm_bp.route('/llm', methods=['POST'])
def llm():
    print("LLm")
    # Обработка POST запроса для передачи промпта в LLM
    data = request.get_json()
    text = data.get('text', '').strip()
    if not text:
        return jsonify({"error": "Текст не указан"}), 400

    try:
        response = send_to_llm(text)
        return jsonify(response)
    except Exception as e:
        return jsonify({"errorik": str(e)}), 500
