from flask import Blueprint, render_template
from flask import current_app as app
from configs import Config

mock_bp = Blueprint('mock', __name__)

@mock_bp.route('/mock')
def index():
    print("MOCK")
    app.logger.info("Rendering TEST request page")
    return render_template('mockPage.html',
                           bridge_url = Config.ROS_BRIDGE_URL)
