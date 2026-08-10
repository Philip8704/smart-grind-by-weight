#!/usr/bin/env python3
"""Serve the local web flasher over HTTPS.

Web Bluetooth is only exposed on a secure context. Over plain http:// to a LAN
address Chrome does not merely block the call - it omits navigator.bluetooth
altogether, so the flasher reports "Web Bluetooth not supported" on every
device including desktop. localhost is the one http exemption, which is why
desktop appears to work only when reached as localhost.

Serving over https fixes it everywhere at once. The certificate is self-signed,
so each device shows an interstitial the first time; accepting it still yields a
secure context and Web Bluetooth becomes available.

Regenerate the certificate if the machine's LAN address changes:

  openssl req -x509 -newkey rsa:2048 -nodes \
    -keyout tools/web-flasher/.certs/key.pem \
    -out tools/web-flasher/.certs/cert.pem -days 825 \
    -subj "/CN=grinder-flasher" \
    -addext "subjectAltName=IP:<lan-ip>,IP:127.0.0.1,DNS:localhost"
"""

import argparse
import http.server
import socket
import ssl
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
FLASHER_DIR = REPO_ROOT / "tools" / "web-flasher"
CERT_DIR = FLASHER_DIR / ".certs"


def detect_lan_ip() -> str:
    """Best-effort LAN address, by asking the routing table which one it would use."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--bind", default="0.0.0.0")
    args = parser.parse_args()

    cert, key = CERT_DIR / "cert.pem", CERT_DIR / "key.pem"
    for path in (cert, key):
        if not path.exists():
            print(f"ERROR: {path} missing - see the header of this file to regenerate")
            return 1

    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=cert, keyfile=key)

    handler = lambda *a, **kw: http.server.SimpleHTTPRequestHandler(
        *a, directory=str(FLASHER_DIR), **kw
    )
    httpd = http.server.ThreadingHTTPServer((args.bind, args.port), handler)
    httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

    print(f"Flasher:  https://{detect_lan_ip()}:{args.port}")
    print(f"Local:    https://localhost:{args.port}")
    print("Expect a certificate warning on first visit - accept it to get a secure context.")
    httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
