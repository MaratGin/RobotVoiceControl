from flask import Blueprint, request, jsonify, current_app, send_from_directory
from services.whisper_service import transcribe_audio
import os
import uuid

audio_bp = Blueprint('audio', __name__)


def _allowed_filename(filename):
    ext = os.path.splitext(filename)[1].lstrip('.').lower()
    return ext in current_app.config.get('ALLOWED_EXTENSIONS', {'webm', 'ogg', 'wav', 'm4a', 'mp3'})


@audio_bp.route('/upload', methods=['POST'])
def upload_audio():
    print("POST /upload called")
    current_app.logger.debug("/upload called")
    try:
        audio_file = request.files.get('audio')
        if not audio_file:
            return jsonify({"error": "Аудиофайл не получен"}), 400

        if not _allowed_filename(audio_file.filename):
            return jsonify({"error": "Неподдерживаемый формат файла"}), 400

        original_ext = os.path.splitext(audio_file.filename)[1].lstrip('.').lower() or 'webm'
        original_filename = f"{uuid.uuid4().hex}.{original_ext}"
        upload_folder = current_app.config['UPLOAD_FOLDER']
        original_path = os.path.join(upload_folder, original_filename)
        audio_file.save(original_path)
        print(f"Saved uploaded file to {original_path}")

        try:
            transcription, wav_filename = transcribe_audio(original_path, upload_folder)
        except Exception as e:
            # ensure we remove the original if processing failed
            current_app.logger.exception("Processing failed for %s: %s", original_filename, e)
            if os.path.exists(original_path):
                os.remove(original_path)
            return jsonify({"error": str(e)}), 500

        return jsonify({
            "original_url": f"/recordings/{original_filename}",
            "wav_url": f"/recordings/{wav_filename}",
            "transcription": transcription
        })

    except Exception as e:
        current_app.logger.exception("Unexpected error in /upload: %s", e)
        return jsonify({"error": str(e)}), 500


@audio_bp.route('/recordings/<filename>')
def get_recording(filename):
    return send_from_directory(current_app.config['UPLOAD_FOLDER'], filename)
