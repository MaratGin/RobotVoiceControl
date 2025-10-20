from flask import Blueprint, render_template
from flask import current_app as app
main_bp = Blueprint('main', __name__)

@main_bp.route('/')
def index():
    print("TEST")
    app.logger.info("Rendering start page")
    return render_template('startPage.html')
