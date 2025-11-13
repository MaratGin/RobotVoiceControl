from pydub import AudioSegment
import os
import uuid
from flask import current_app
import whisper


def _load_model_if_needed():
    """Загружает модель Whisper в конфиг, если она ещё не загружена."""
    if current_app.config.get('WHISPER_MODEL_INSTANCE') is None:
        model_name = current_app.config.get('WHISPER_MODEL', 'base')
        current_app.logger.info(f"Loading Whisper model: {model_name}")
        model = whisper.load_model(model_name)
        current_app.config['WHISPER_MODEL_INSTANCE'] = model
        current_app.logger.info("Whisper model loaded")


def transcribe_audio(file_path, upload_folder):
    """Конвертирует входной файл в WAV и расшифровывает с помощью Whisper.
    """
    print("rewrewrewrwer")

    current_app.logger.debug("TRANSCRIBE AUDIO SERVICE: %s", file_path)

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"input file not found: {file_path}")

    wav_filename = f"{uuid.uuid4().hex}.wav"
    wav_path = os.path.join(upload_folder, wav_filename)

    # Convert to WAV
    try:
        audio = AudioSegment.from_file(file_path)
        audio = audio.set_frame_rate(16000).set_channels(1)
        audio.export(wav_path, format="wav")
    except Exception as e:
        current_app.logger.exception("Audio conversion failed: %s", e)
        # Clean up partial files
        if os.path.exists(wav_path):
            os.remove(wav_path)
        raise

    # Ensure model is loaded
    _load_model_if_needed()
    print("model loaded")


    model = current_app.config.get('WHISPER_MODEL_INSTANCE')
    if model is None:
        raise RuntimeError("Whisper model is not available")
    print("Starting transcription")
    # Transcribe
    try:
        result = model.transcribe(wav_path, language="ru", fp16=False)
        transcription = result.get("text", "").strip()
        current_app.logger.info("Transcription finished: %s", transcription[:80])
        return transcription, wav_filename
    except Exception as e:
        current_app.logger.exception("Whisper transcription failed: %s", e)
        # Cleanup on failure
        if os.path.exists(wav_path):
            os.remove(wav_path)
        raise


# def transcribe_audio(file_path, upload_folder):
#     print("TRANSCRIBE AUDIO SERVICE")
#     wav_filename = f"{uuid.uuid4().hex}.wav"
#     wav_path = os.path.join(upload_folder, wav_filename)

#     audio = AudioSegment.from_file(file_path)
#     audio.export(wav_path, format="wav", parameters=["-ar", "16000", "-ac", "1"])
#     model = current_app.config['WHISPER_MODEL_INSTANCE']
#     result = model.transcribe(wav_path, language="ru")
#     transcription = result["text"].strip()
#     return transcription, wav_filename
