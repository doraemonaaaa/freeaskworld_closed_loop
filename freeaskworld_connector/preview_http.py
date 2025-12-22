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

            if self.path in ("/mjpeg", "/stream"):
                boundary = b"--frame"
                self.send_response(200)
                self.send_header(
                    "Content-Type", f"multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    last_frame = None
                    while True:
                        frame = get_frame()
                        if frame is None:
                            time.sleep(0.1)  # Wait longer if no frame
                            continue
                        # Only send if frame changed (avoid redundant sends)
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
                        time.sleep(0.05)  # ~20fps max for preview
                except (BrokenPipeError, ConnectionResetError):
                    return

            if self.path in ("/viewer", "/index.html"):
                html = b"""<!doctype html><html><body style='margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh'>
<img src='/mjpeg' style='max-width:100%;max-height:100%;object-fit:contain;' />
</body></html>"""
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
    # Load config
    try:
        from .config import config
        host = config.PREVIEW_HOST
        port = config.PREVIEW_PORT
    except ImportError:
        import os
        host = os.environ.get("PREVIEW_HOST", host)
        port = int(os.environ.get("PREVIEW_PORT", str(port)))
    
    handler_cls = _make_handler(get_frame)
    class ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        daemon_threads = True
        allow_reuse_address = True

    httpd = ThreadingTCPServer((host, port), handler_cls)

    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd, thread


__all__ = ["start_preview_server"]
