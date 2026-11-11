"""Live web view of the robot camera and what our navigation code sees.

Run on the Jetson in a prepared terminal while the base is running (and our
navigation node, for the right-hand panel):

    python3 ~/a2r-student/robot/src/team_navigation/team_navigation/stream.py

Then open http://localhost:8081/ in your laptop browser, having connected
with an SSH tunnel: ssh -L 8081:localhost:8081 a2r-lab-jetson@ROBOT_IP
(or http://ROBOT_IP:8081/ directly, if the browser may reach the robot LAN).
Left: camera, with the median HSV colour of the boxed bottom-centre region
(compare with yellow_hsv_low/high in config/navigation.yaml).
Right: navigation/debug_image (magenta = counted as yellow, green = trail,
white circle = steering target, top line = status and request).
Ctrl+C to stop. A viewing tool only; it never sends motion.
"""
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

PORT = 8081
FPS = 10
PANEL_H = 360
STALE_S = 1.

PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot view</title><style>
body{margin:0;background:#111;color:#ddd;font:14px system-ui,sans-serif}
img{width:100%;max-width:1280px;display:block;margin:auto}
p{max-width:1280px;margin:8px auto;padding:0 12px}
</style></head><body><img src="/stream">
<p>Left: camera + bottom-centre HSV. Right: magenta = yellow, green = trail,
white circle = target.</p></body></html>"""


def label(image, text, y=20):
    for color, thickness in (((0, 0, 0), 4), ((255, 255, 255), 1)):
        cv2.putText(image, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    color, thickness, cv2.LINE_AA)


def placeholder(width, text):
    image = np.zeros((PANEL_H, width, 3), np.uint8)
    label(image, text, PANEL_H // 2)
    return image


def fit(image):
    h, w = image.shape[:2]
    return cv2.resize(image, (round(w * PANEL_H / h), PANEL_H),
                      interpolation=cv2.INTER_AREA if h > PANEL_H else cv2.INTER_NEAREST)


class Viewer(Node):
    def __init__(self, namespace):
        super().__init__('stream_viewer')
        self.bridge = CvBridge()
        self.latest = {}
        self.jpeg = None
        self.lock = threading.Lock()
        for name, topic in (('camera', 'camera/image_raw'),
                            ('debug', 'navigation/debug_image')):
            self.create_subscription(
                Image, f'/{namespace}/{topic}',
                lambda msg, name=name: self.latest.__setitem__(name, (time.monotonic(), msg)),
                qos_profile_sensor_data)
        self.create_timer(1. / FPS, self.compose)

    def frame(self, name):
        entry = self.latest.get(name)
        if entry is None or time.monotonic() - entry[0] > STALE_S:
            return None
        return self.bridge.imgmsg_to_cv2(entry[1], desired_encoding='bgr8')

    def camera_panel(self):
        image = self.frame('camera')
        if image is None:
            return placeholder(640, 'no camera (is the base running?)')
        image = fit(image)
        h, w = image.shape[:2]
        x0, x1, y0 = int(w * .4), int(w * .6), int(h * .7)
        hsv = cv2.cvtColor(image[y0:h, x0:x1], cv2.COLOR_BGR2HSV).reshape(-1, 3)
        H, S, V = np.median(hsv, axis=0).astype(int)
        cv2.rectangle(image, (x0, y0), (x1, h - 1), (255, 255, 255), 1)
        label(image, f'box HSV: H {H}  S {S}  V {V}')
        return image

    def debug_panel(self, width):
        image = self.frame('debug')
        if image is None:
            return placeholder(width, 'no debug image (is our navigation running?)')
        return fit(image)

    def compose(self):
        left = self.camera_panel()
        right = self.debug_panel(left.shape[1])
        ok, jpeg = cv2.imencode('.jpg', np.hstack([left, right]),
                                [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock:
                self.jpeg = jpeg.tobytes()


def make_handler(viewer):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with viewer.lock:
                            jpeg = viewer.jpeg
                        if jpeg:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                             b'Content-Length: %d\r\n\r\n' % len(jpeg))
                            self.wfile.write(jpeg + b'\r\n')
                        time.sleep(1. / FPS)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def log_message(self, *args):
            pass
    return Handler


def robot_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('192.168.0.1', 9))  # no packet is sent; picks the LAN address
            return s.getsockname()[0]
    except OSError:
        return 'ROBOT_IP'


def main():
    namespace = os.environ.get('ROBOT_NAMESPACE')
    if not namespace:
        raise SystemExit('ROBOT_NAMESPACE is not set: source setup_terminal.sh first')
    rclpy.init()
    viewer = Viewer(namespace)
    server = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(viewer))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Viewer running. Open in your laptop browser:\n'
          f'  via SSH tunnel (ssh -L {PORT}:localhost:{PORT} ...):  http://localhost:{PORT}/\n'
          f'  or directly on the robot Wi-Fi:                       http://{robot_ip()}:{PORT}/\n'
          f'Ctrl+C to stop.')
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
