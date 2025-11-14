import os
import logging
from flask import Flask
from configs import Config

# from app.config import Config

# app = Flask(__name__)
# UPLOAD_FOLDER = 'recordings'
# os.makedirs(UPLOAD_FOLDER, exist_ok=True)


def create_app():
    app = Flask(__name__)
    app.logger.setLevel(logging.INFO)
    app.config.from_object(Config)
    # Создаём папку для записей
    os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

    # # Регистрируем блюпринты
    from routes.main_route import main_bp
    from routes.audio_route import audio_bp
    from routes.llm_route import llm_bp
    from routes.mock_route import mock_bp
    app.register_blueprint(main_bp)
    app.register_blueprint(audio_bp)
    app.register_blueprint(llm_bp)
    app.register_blueprint(mock_bp)
    # Defer heavy model loading until first request to speed up startup.
    app.config.setdefault('WHISPER_MODEL_INSTANCE', None)
    app.logger.info("App created, whisper model will be loaded lazily on first request")
    return app


if __name__ == '__main__':
    app = create_app()
    print("🚀 Сервер запущен: http://localhost:5005")
    print("📁 Записи сохраняются в папку 'recordings'")
    print("🎯 Whisper готов к распознаванию речи")
    app.run(host='0.0.0.0', port=5005, debug=True)