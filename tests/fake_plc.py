"""A stand-in ControlByWeb X-432 for tests: a local web server with its API.

It serves what the X-400 series serves -- `state.json` and `state.xml` with
relay1..relay16 and digitalInput1..digitalInput18, and `?relayN=0|1` to set a
relay -- so the brake driver can be exercised end to end over real HTTP
without the device.

`wire(relay, input, invert)` makes an input follow a relay, which is how a
test models "the brake's feedback switch": turn the relay on and the input
changes as the brake would. Leave it unwired and the input just sits there,
which is what an unconnected input does.
"""

from __future__ import annotations

import base64
import json
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional, Tuple


class FakeX432:
    def __init__(self, relays: int = 16, inputs: int = 18,
                 password: str = "", username: str = "admin",
                 serve_json: bool = True):
        self.relays: Dict[int, int] = {n: 0 for n in range(1, relays + 1)}
        self.inputs: Dict[int, int] = {n: 0 for n in range(1, inputs + 1)}
        self.password = password
        self.username = username
        self.serve_json = serve_json
        #: relay number -> (input number, inverted)
        self.wiring: Dict[int, Tuple[int, bool]] = {}
        #: Relays the "PLC's own logic" forces, whatever is asked.
        self.stuck: Dict[int, int] = {}
        self.requests: List[str] = []
        #: The next this-many requests are answered only after `slow_s`
        #: seconds, longer than the client waits: a busy or flaky PLC.
        self.slow_next = 0
        self.slow_s = 2.0
        self._lock = threading.Lock()
        self._server: Optional[ThreadingHTTPServer] = None

    # ------------------------------------------------------------ fixtures

    def wire(self, relay: int, input_number: int, invert: bool = False) -> None:
        with self._lock:
            self.wiring[relay] = (input_number, invert)
            self._propagate()

    def _propagate(self) -> None:
        for relay, (input_number, invert) in self.wiring.items():
            value = self.relays[relay]
            self.inputs[input_number] = (1 - value) if invert else value

    def payload(self) -> Dict[str, object]:
        with self._lock:
            data: Dict[str, object] = {}
            for n, v in self.relays.items():
                data[f"relay{n}"] = v
            for n, v in self.inputs.items():
                data[f"digitalInput{n}"] = v
            data["serialNumber"] = "00:0C:C8:FA:KE:00"
            return data

    # -------------------------------------------------------------- server

    def start(self) -> "FakeX432":
        plc = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):   # keep test output quiet
                return

            def do_GET(self):
                plc.requests.append(self.path)
                with plc._lock:
                    slow = plc.slow_next > 0
                    if slow:
                        plc.slow_next -= 1
                if slow:
                    time.sleep(plc.slow_s)
                if plc.password:
                    expected = "Basic " + base64.b64encode(
                        f"{plc.username}:{plc.password}".encode()).decode()
                    if self.headers.get("Authorization") != expected:
                        self.send_response(401)
                        self.send_header("WWW-Authenticate", 'Basic realm="x432"')
                        self.end_headers()
                        return
                parsed = urllib.parse.urlparse(self.path)
                page = parsed.path.lstrip("/")
                if page == "state.json" and not plc.serve_json:
                    self.send_response(404)
                    self.end_headers()
                    return
                if page not in ("state.json", "state.xml"):
                    self.send_response(404)
                    self.end_headers()
                    return
                query = urllib.parse.parse_qs(parsed.query)
                with plc._lock:
                    for key, values in query.items():
                        if key.startswith("relay") and key[5:].isdigit():
                            n = int(key[5:])
                            if n in plc.relays:
                                plc.relays[n] = 1 if values[-1] in ("1", "on") else 0
                    for n, v in plc.stuck.items():
                        plc.relays[n] = v
                    plc._propagate()
                data = plc.payload()
                if page == "state.json":
                    body = json.dumps(data).encode()
                    kind = "application/json"
                else:
                    body = ("<datavalues>" + "".join(
                        f"<{k}>{v}</{k}>" for k, v in data.items())
                        + "</datavalues>").encode()
                    kind = "text/xml"
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        # A client that gave up on a slow reply closes its end; writing the
        # reply then fails, which is expected here and not worth a traceback.
        self._server.handle_error = lambda request, client_address: None
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        return self

    @property
    def port(self) -> int:
        return self._server.server_address[1]

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    def __enter__(self) -> "FakeX432":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
