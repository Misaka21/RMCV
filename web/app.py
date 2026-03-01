"""
RMCV Web Dashboard - Flask Server
Use dashboard module for data sharing

Start: ./RMCV2026 --web
"""

import time
import threading
import queue
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

# msgpack (可选，降级为 JSON)
try:
    import msgpack
    HAS_MSGPACK = True
    print("[WebServer] msgpack loaded")
except ImportError:
    HAS_MSGPACK = False
    print("[WebServer] msgpack not available, WebSocket will use JSON fallback")

# flask-sock (可选，降级为 SSE only)
try:
    from flask_sock import Sock
    sock = Sock(app)
    HAS_WEBSOCKET = True
    print("[WebServer] flask-sock loaded")
except ImportError:
    HAS_WEBSOCKET = False
    print("[WebServer] flask-sock not available, SSE only mode")


def get_dashboard_data():
    """Get dashboard data from dashboard module"""
    if DASHBOARD_MODULE is not None:
        try:
            return DASHBOARD_MODULE.all()
        except Exception as e:
            print(f"[WebServer] Error reading dashboard: {e}")
    return {}


def get_dashboard_version():
    """Get dashboard data version counter"""
    if DASHBOARD_MODULE is not None:
        try:
            return DASHBOARD_MODULE.version()
        except Exception:
            pass
    return 0


# ======================== WebSocket 快照线程 ========================
# 200Hz 读取 C++ Registry，打包为 msgpack 二进制，推入队列
# WebSocket 线程从队列取数据发送 (纯 I/O，GIL 极短)

_ws_clients = []  # type: list[queue.Queue]
_ws_clients_lock = threading.Lock()


def _snapshot_worker():
    """快照线程: 200Hz 采样 C++ Registry, 分发给所有 WebSocket 客户端"""
    last_ver = 0
    while True:
        ver = get_dashboard_version()
        if ver != last_ver:
            last_ver = ver
            data = get_dashboard_data()

            if HAS_MSGPACK:
                packed = msgpack.packb(data, use_bin_type=True)
            else:
                import json
                packed = json.dumps(data).encode('utf-8')

            with _ws_clients_lock:
                for q in _ws_clients:
                    try:
                        q.put_nowait(packed)
                    except queue.Full:
                        # 丢弃旧帧，保留最新
                        try:
                            q.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            q.put_nowait(packed)
                        except queue.Full:
                            pass
        time.sleep(0.005)  # 200Hz


# 启动快照线程
threading.Thread(target=_snapshot_worker, daemon=True).start()


@app.route('/')
def index():
    return render_template('dashboard.html')


@app.route('/api/data')
def get_data():
    """Get all dashboard data"""
    return jsonify(get_dashboard_data())


@app.route('/api/stream')
def stream():
    """SSE real-time data stream (降级方案)"""
    def generate():
        import json
        try:
            while True:
                data = get_dashboard_data()
                yield f"data: {json.dumps(data)}\n\n"
                time.sleep(0.01)  # 100Hz
        except GeneratorExit:
            pass

    return Response(generate(), mimetype='text/event-stream')


# ======================== WebSocket 路由 ========================
if HAS_WEBSOCKET:
    @sock.route('/ws/data')
    def data_ws(ws):
        """WebSocket 二进制数据推送"""
        client_queue = queue.Queue(maxsize=2)
        with _ws_clients_lock:
            _ws_clients.append(client_queue)
        try:
            # 告知前端序列化格式
            ws.send(b'\x01' if HAS_MSGPACK else b'\x00')
            while True:
                packed = client_queue.get()
                ws.send(packed)
        except Exception:
            pass
        finally:
            with _ws_clients_lock:
                try:
                    _ws_clients.remove(client_queue)
                except ValueError:
                    pass


@app.route('/api/topics')
def list_topics():
    """List all image topics (only topics starting with '/' are for Web UI)"""
    if MESSAGE_MODULE is not None:
        try:
            all_topics = list(MESSAGE_MODULE.names())
            topics = [t for t in all_topics if t.startswith('/')]
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
            import cv2
            try:
                sub = MESSAGE_MODULE.Subscriber(topic, 1)
                while True:
                    try:
                        mat = sub.pop_for(2000)
                        if not mat.empty():
                            jpeg_code = cv2.imencode(".jpeg", mat.get_nparray())[1].tobytes()
                            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg_code + b'\r\n\r\n'
                    except Exception:
                        time.sleep(0.1)
            except GeneratorExit:
                pass
            except Exception as e:
                print(f"[WebServer] Video error: {e}")
        else:
            try:
                while True:
                    yield_placeholder_frame(topic)
                    time.sleep(0.5)
            except GeneratorExit:
                pass

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
        "websocket": HAS_WEBSOCKET,
        "msgpack": HAS_MSGPACK,
        "topics": list(MESSAGE_MODULE.names()) if MESSAGE_MODULE else []
    })


if __name__ == '__main__':
    print(f"[WebServer] Dashboard: {'UMT ObjManager' if DASHBOARD_MODULE else 'Mock data'}")
    print(f"[WebServer] Video: {'Message_cvMat' if MESSAGE_MODULE else 'Not available'}")
    print(f"[WebServer] Transport: {'WebSocket+msgpack' if HAS_WEBSOCKET and HAS_MSGPACK else 'WebSocket+JSON' if HAS_WEBSOCKET else 'SSE only'}")
    print(f"[WebServer] Starting on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
