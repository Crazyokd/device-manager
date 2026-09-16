#!/usr/bin/env python3
import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse


PARAMETERS = {
    "demo_imu": {
        "device.enable": {"type": "bool", "value": False},
        "device.interface.serial_port": {"type": "string", "value": "/dev/ttyUSB0"},
        "device.interface.serial_baudrate": {"type": "int32", "value": 115200},
        "imu.frame_id": {"type": "string", "value": "imu_link"},
    },
}


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path != "/parameters":
            self.send_error(404)
            return

        query = parse_qs(parsed.query)
        device_id = query.get("device_id", [""])[0]
        default = {"device.enable": {"type": "bool", "value": False}}
        body = json.dumps({"data": {"items": PARAMETERS.get(device_id, default)}}).encode()

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


if __name__ == "__main__":
    port = int(os.environ.get("PORT", "3000"))
    HTTPServer(("127.0.0.1", port), Handler).serve_forever()
