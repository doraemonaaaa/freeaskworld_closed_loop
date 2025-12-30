"""Minimal HTTP JPEG preview server for the latest WebRTC frame."""

from __future__ import annotations

import http.server
import socketserver
import threading
import time
from typing import Callable, Optional

HandlerFactory = Callable[[], http.server.BaseHTTPRequestHandler]
GetFrameFn = Callable[[], Optional[bytes]]


def _make_handler(get_frame: GetFrameFn) -> HandlerFactory:
    class FrameHandler(http.server.BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - HTTP method name
            # Single JPEG
            if self.path in ("/", "/frame", "/frame.jpg"):
                frame = get_frame()
                if frame is None:
                    self.send_response(503)
                    self.end_headers()
                    self.wfile.write(b"no frame available")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                return

            # MJPEG stream
            if self.path in ("/mjpeg", "/stream"):
                boundary = b"--frame"
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    last_frame = None
                    while True:
                        frame = get_frame()
                        if frame is None:
                            time.sleep(0.1)
                            continue
                        if frame == last_frame:
                            time.sleep(0.05)
                            continue
                        last_frame = frame
                        self.wfile.write(boundary + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        )
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    return

            # Viewer page with FPS HUD
            if self.path in ("/viewer", "/index.html"):
                html = b"""<!doctype html>
<html>
<body style='margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh;position:relative;'>
  <img id='stream' src='/mjpeg' style='max-width:100%;max-height:100%;object-fit:contain;' />
  <div id='hud' style='position:absolute;top:10px;left:10px;padding:6px 10px;border-radius:6px;background:rgba(0,0,0,0.5);color:#eee;font-family:Arial,Helvetica,sans-serif;font-size:14px;'>
    FPS: <span id='fps'>--</span>
  </div>
  <img id='probe' style='display:none;' />
  <script>
    (function() {
      const fpsEl = document.getElementById('fps');
      const probe = document.getElementById('probe');
      let count = 0;
      let start = performance.now();

      function scheduleNext() {
        probe.src = '/frame?cache=' + performance.now();
      }

      probe.onload = function() {
        count += 1;
        const now = performance.now();
        const elapsed = now - start;
        if (elapsed >= 1000) {
          const fps = (count * 1000 / elapsed).toFixed(1);
          fpsEl.textContent = fps;
          count = 0;
          start = now;
        }
        scheduleNext();
      };

      probe.onerror = scheduleNext;
      scheduleNext();
    })();
  </script>
</body>
</html>"""
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return

            self.send_response(404)
            self.end_headers()

        def log_message(self, format: str, *args) -> None:  # type: ignore[override]
            return  # silence default logging

    return FrameHandler


def start_preview_server(get_frame: GetFrameFn, host: str = "0.0.0.0", port: int = 8080):
    handler_cls = _make_handler(get_frame)
    class ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        daemon_threads = True
        allow_reuse_address = True

    httpd = ThreadingTCPServer((host, port), handler_cls)

    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd, thread


__all__ = ["start_preview_server"]
