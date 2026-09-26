"""Tiny browser view for a calibration session: latest annotated frame + status text."""

from __future__ import annotations

import http.server
import json
import socketserver
import threading

_PAGE = b"""<!doctype html><meta charset=utf-8><title>calibration live view</title>
<style>body{font:14px system-ui;background:#111;color:#ddd;margin:16px}
img{max-width:100%;border:1px solid #444}pre{white-space:pre-wrap;background:#1b1b1b;padding:8px}</style>
<h3>ESP32-S3 camera calibration &mdash; live view</h3>
<img id=im src=/latest.jpg><pre id=st>waiting...</pre>
<script>
async function tick(){
  try{const s=await (await fetch('/status.json',{cache:'no-store'})).json();
      document.getElementById('st').textContent=s.text;
      if(s.version!==window._v){window._v=s.version;
        document.getElementById('im').src='/latest.jpg?v='+s.version;}}catch(e){}
  setTimeout(tick,500);}
tick();
</script>"""


class LiveView:
    def __init__(self, port: int | None):
        self.jpeg: bytes = b""
        self.text = "waiting for the first frame..."
        self.version = 0
        self._lock = threading.Lock()
        self._srv = None
        if port:
            view = self

            class H(http.server.BaseHTTPRequestHandler):
                def log_message(self, *a):
                    pass

                def do_GET(self):
                    with view._lock:
                        jpeg, text, ver = view.jpeg, view.text, view.version
                    if self.path.startswith("/latest.jpg"):
                        body, ctype = jpeg, "image/jpeg"
                    elif self.path.startswith("/status.json"):
                        body, ctype = json.dumps({"text": text, "version": ver}).encode(), "application/json"
                    else:
                        body, ctype = _PAGE, "text/html"
                    self.send_response(200)
                    self.send_header("Content-Type", ctype)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)

            class S(socketserver.ThreadingMixIn, http.server.HTTPServer):
                daemon_threads = True
                allow_reuse_address = True

            self._srv = S(("0.0.0.0", port), H)
            threading.Thread(target=self._srv.serve_forever, daemon=True).start()

    def update(self, jpeg: bytes | None = None, text: str | None = None) -> None:
        with self._lock:
            if jpeg is not None:
                self.jpeg = jpeg
                self.version += 1
            if text is not None:
                self.text = text

    def update_bgr(self, img, text: str | None = None, max_w: int = 960) -> None:
        import cv2
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if img.shape[1] > max_w:
            s = max_w / img.shape[1]
            img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            self.update(buf.tobytes(), text)

    def close(self) -> None:
        if self._srv:
            self._srv.shutdown()
            self._srv.server_close()
