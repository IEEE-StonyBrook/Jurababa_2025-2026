"""Send a command through a running serial_term.py and print the reply.

Examples:
  python tools/serial_cmd.py BAT
  python tools/serial_cmd.py "STEP 3 1000" --wait 10
  python tools/serial_cmd.py OL --wait 60 --idle 5
  python tools/serial_cmd.py --wait 5            # just listen

Collection stops after --wait seconds, or earlier once output has gone quiet
for --idle seconds (long trials need a larger --idle).
"""

import argparse
import socket
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("command", nargs="*", help="command(s) to send, one per argument")
parser.add_argument("--wait", type=float, default=3.0, help="max seconds to collect output")
parser.add_argument("--idle", type=float, default=0.8, help="stop after this many quiet seconds")
parser.add_argument("--port", type=int, default=7777)
args = parser.parse_args()

try:
    sock = socket.create_connection(("127.0.0.1", args.port), timeout=2)
except OSError:
    sys.exit("serial_term.py is not running (start it with: python tools/serial_term.py)")

for cmd in args.command:
    sock.sendall(cmd.encode("utf-8") + b"\n")

sock.settimeout(0.1)
out = bytearray()
start = last_rx = time.monotonic()
while True:
    now = time.monotonic()
    if now - start > args.wait:
        break
    if out and now - last_rx > args.idle:
        break
    try:
        data = sock.recv(4096)
        if not data:
            break
        out += data
        last_rx = time.monotonic()
    except socket.timeout:
        pass
sock.close()

sys.stdout.write(out.decode("utf-8", errors="replace").replace("\r\n", "\n"))
