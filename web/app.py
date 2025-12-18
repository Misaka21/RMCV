"""
RMCV Web Dashboard - Flask Server
Use dashboard module for data sharing

Start: ./RMCV2026 --py web/app.py
"""

import time
from flask import Flask, render_template, jsonify, Response

app = Flask(__name__)

# Try to import pybind11 embedded modules
DASHBOARD_MODULE = None
MESSAGE_MODULE = None

try:
    import dashboard
    DASHBOARD_MODULE = dashboard
    print("[WebServer] dashboard module loaded")
except ImportError as e:
    print(f"[WebServer] dashboard module not available: {e}")

try:
    import Message_cvMat
    MESSAGE_MODULE = Message_cvMat
    print("[WebServer] Message_cvMat loaded")
except ImportError as e:
    print(f"[WebServer] Message_cvMat not available: {e}")


def get_dashboard_data():
    """Get dashboard data from dashboard module"""
    if DASHBOARD_MODULE is not None:
        try:
            return DASHBOARD_MODULE.all()
        except Exception as e:
            print(f"[WebServer] Error reading dashboard: {e}")
    return {}


@app.route('/')
def index():
    return render_template('dashboard.html')


@app.route('/api/data')
def get_data():
    """Get all dashboard data"""
    return jsonify(get_dashboard_data())


@app.route('/api/stream')
def stream():
    """SSE real-time data stream"""
    def generate():
        import json
        while True:
            data = get_dashboard_data()
            yield f"data: {json.dumps(data)}\n\n"
            time.sleep(0.01)  # 100Hz

    return Response(generate(), mimetype='text/event-stream')


@app.route('/api/topics')
def list_topics():
    """List all image topics"""
    if MESSAGE_MODULE is not None:
        try:
            topics = list(MESSAGE_MODULE.names())
            return jsonify(topics)
        except Exception as e:
            return jsonify({"error": str(e)})
    return jsonify([])


@app.route('/api/video/<path:topic>')
def video_feed(topic):
    """MJPEG video stream"""
    topic = "/" + topic  # Restore leading slash

    def generate():
        if MESSAGE_MODULE is not None:
            try:
                sub = MESSAGE_MODULE.Subscriber(topic, 1)
                while True:
                    try:
                        mat = sub.pop_for(1000)
                        if not mat.empty():
                            jpeg = mat.to_jpeg(80)
                            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
                    except Exception:
                        pass
            except Exception as e:
                print(f"[WebServer] Video error: {e}")
                yield_error_frame(f"Error: {str(e)[:40]}")
        else:
            while True:
                yield_placeholder_frame(topic)
                time.sleep(0.5)

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


def yield_error_frame(msg):
    """Generate error frame"""
    import cv2
    import numpy as np
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, msg, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    _, jpeg = cv2.imencode('.jpg', img)
    return b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'


def yield_placeholder_frame(topic):
    """Generate placeholder frame"""
    import cv2
    import numpy as np
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, f"Topic: {topic}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(img, "Waiting for data...", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
    _, jpeg = cv2.imencode('.jpg', img)
    return b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'


@app.route('/api/status')
def status():
    """System status"""
    return jsonify({
        "dashboard_module": DASHBOARD_MODULE is not None,
        "message_module": MESSAGE_MODULE is not None,
        "topics": list(MESSAGE_MODULE.names()) if MESSAGE_MODULE else []
    })


if __name__ == '__main__':
    print(f"[WebServer] Dashboard: {'UMT ObjManager' if DASHBOARD_MODULE else 'Mock data'}")
    print(f"[WebServer] Video: {'Message_cvMat' if MESSAGE_MODULE else 'Not available'}")
    print(f"[WebServer] Starting on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
