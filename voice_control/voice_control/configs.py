class Config:
    UPLOAD_FOLDER = 'recordings'  # куда сохраняются голосовые записи
    OLLAMA_SERVER = 'http://192.168.0.105:11434'  # IP для обращения к ллмке
    WHISPER_MODEL = 'base'  # тип модели виспера
    # Limit uploads to 16 MB by default
    MAX_CONTENT_LENGTH = 16 * 1024 * 1024
    # Allowed audio extensions accepted from the browser
    ALLOWED_EXTENSIONS = {'webm', 'ogg', 'wav', 'm4a', 'mp3'}
