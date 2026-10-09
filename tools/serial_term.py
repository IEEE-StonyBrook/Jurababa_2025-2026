"""Shared serial terminal: a simple CoolTerm-style window that owns the COM
port and also exposes it on a localhost socket, so a script (e.g. Claude via
serial_cmd.py) can send commands and read output while you watch and type.

Windows lets only one process open a COM port, so this window is the single
owner; everything else talks through it.

Run:   python tools/serial_term.py
Link:  127.0.0.1:7777 (line-based: each line received is sent to the device,
       everything the device prints is streamed back to every client)
Logs:  tools/logs/latest.log (+ a timestamped copy per session)
"""

import argparse
import datetime
import pathlib
import queue
import select
import socket
import socketserver
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

LINK_HOST = "127.0.0.1"
LINK_PORT = 7777
BAUD_RATES = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
PICO_VID = 0x2E8A
LINE_ENDING = b"\r\n"

LOG_DIR = pathlib.Path(__file__).resolve().parent / "logs"


def parse_args():
    p = argparse.ArgumentParser(description="Shared serial terminal.")
    p.add_argument("--port", help="COM port to open at startup (default: auto-select the Pico)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--self-test", action="store_true",
                   help="run the [DIAG] filter checks and exit (no hardware needed)")
    p.add_argument("--link-port", type=int, default=LINK_PORT,
                   help="localhost port serial_cmd.py connects to; change it to run a second window")
    return p.parse_args()


ARGS = parse_args()
LINK_PORT = ARGS.link_port


DIAG_TAG = "[DIAG]"


class DiagFilter:
    """Hides [DIAG] lines from the window; the log file still gets everything.

    Serial arrives in arbitrary chunks, so a line can be split anywhere, and the CLI
    prompt "> " carries no trailing newline. A partial tail is therefore held until it
    completes, and only released by flush_idle() once the stream goes quiet and the
    tail provably cannot become a [DIAG] line. Releasing clears the buffer, so a later
    chunk starts a fresh tag check instead of merging with text already shown.
    """

    def __init__(self):
        self.pending = ""

    def feed(self, text):
        """Returns the portion of complete lines that should be displayed."""
        self.pending += text
        out = []
        while chr(10) in self.pending:
            line, _, self.pending = self.pending.partition(chr(10))
            if not self._is_diag(line):
                out.append(line + chr(10))
        return "".join(out)

    def flush_idle(self):
        """Releases a held partial line that cannot be a [DIAG] line, e.g. the prompt."""
        if not self.pending or self._ambiguous(self.pending):
            return ""
        out, self.pending = self.pending, ""
        return out

    @staticmethod
    def _key(text):
        return text.lstrip(chr(13) + chr(10) + " " + chr(9))

    @classmethod
    def _is_diag(cls, line):
        return cls._key(line).startswith(DIAG_TAG)

    @classmethod
    def _ambiguous(cls, partial):
        key = cls._key(partial)
        return key.startswith(DIAG_TAG) or DIAG_TAG.startswith(key)


class Log:
    def __init__(self):
        LOG_DIR.mkdir(exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.files = [
            open(LOG_DIR / f"serial_{stamp}.log", "a", encoding="utf-8", newline=""),
            open(LOG_DIR / "latest.log", "w", encoding="utf-8", newline=""),
        ]
        self.start = time.monotonic()
        self.at_line_start = True

    def write(self, text):
        # Stamp each line with ms since session start. Stalls and loop jitter are
        # timing faults, so the raw byte stream alone cannot measure them. Serial
        # arrives in arbitrary chunks, so track whether we are mid-line.
        out = []
        for part in text.splitlines(keepends=True):
            if self.at_line_start:
                out.append(f"[{(time.monotonic() - self.start) * 1000.0:10.1f}] ")
            out.append(part)
            self.at_line_start = part.endswith((chr(10), chr(13)))
        stamped = "".join(out)
        for f in self.files:
            f.write(stamped)
            f.flush()


class SerialLink:
    """Owns the serial port on a background thread; reconnects after the
    device drops (e.g. reflashing the Pico) until disconnect() is called."""

    def __init__(self, events):
        self.events = events  # queue of (kind, payload) for the GUI thread
        self.ser = None
        self.port = None
        self.baud = 115200
        self.want_connected = False
        self.lock = threading.Lock()
        self.clients = []
        self.clients_lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def connect(self, port, baud):
        self.disconnect()
        self.port, self.baud = port, baud
        self.want_connected = True

    def disconnect(self):
        self.want_connected = False
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        self.events.put(("status", "disconnected"))

    def set_baud(self, baud):
        self.baud = baud
        with self.lock:
            if self.ser:
                self.ser.baudrate = baud

    def send(self, text, source):
        with self.lock:
            ok = self.ser is not None
            if ok:
                try:
                    self.ser.write(text.encode("utf-8") + LINE_ENDING)
                except Exception:
                    ok = False
        self.events.put(("sent", (text, source, ok)))
        return ok

    def _broadcast(self, data):
        # Snapshot under the lock, then send outside it. Holding clients_lock
        # across a blocking sendall deadlocks the whole terminal: the link
        # handler needs the same lock to register a client, so one client that
        # stops reading wedges command delivery AND this reader thread, which
        # silently stops all logging. select() keeps a stalled client from
        # blocking us at all; a diagnostic mirror may drop bytes, not stall.
        with self.clients_lock:
            targets = list(self.clients)
        dead = []
        for c in targets:
            try:
                if not select.select([], [c], [], 0)[1]:
                    continue
                c.sendall(data)
            except OSError:
                dead.append(c)
        if dead:
            with self.clients_lock:
                for c in dead:
                    if c in self.clients:
                        self.clients.remove(c)

    def _run(self):
        waiting_reported = False
        while True:
            if not self.want_connected:
                time.sleep(0.05)
                continue
            if self.ser is None:
                try:
                    s = serial.Serial(self.port, self.baud, timeout=0.05)
                    s.dtr = True  # Pico USB CDC only transmits once DTR is asserted
                    s.rts = True
                    with self.lock:
                        if not self.want_connected:
                            s.close()
                            continue
                        self.ser = s
                    waiting_reported = False
                    self.events.put(("status", f"connected {self.port} @ {self.baud}"))
                except Exception:
                    if not waiting_reported:
                        self.events.put(("status", f"waiting for {self.port}..."))
                        waiting_reported = True
                    time.sleep(0.5)
                continue
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                with self.lock:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                self.events.put(("status", f"lost {self.port} - reconnecting..."))
                continue
            if data:
                self.events.put(("rx", data.decode("utf-8", errors="replace")))
                self._broadcast(data)


def start_link_server(link):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            with link.clients_lock:
                link.clients.append(self.request)
            try:
                for raw in self.rfile:
                    link.send(raw.decode("utf-8", errors="replace").rstrip("\r\n"), "claude")
            finally:
                with link.clients_lock:
                    if self.request in link.clients:
                        link.clients.remove(self.request)

    class Server(socketserver.ThreadingTCPServer):
        daemon_threads = True
        allow_reuse_address = True

    server = Server((LINK_HOST, LINK_PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class App:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.log = Log()
        self.link = SerialLink(self.events)
        self.diag = DiagFilter()
        self.history, self.history_pos = [], 0

        root.title("Serial Terminal")
        root.geometry("900x600")

        bar = ttk.Frame(root, padding=6)
        bar.pack(fill="x")
        ttk.Label(bar, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=34, state="readonly")
        self.port_box.pack(side="left", padx=4)
        ttk.Button(bar, text="⟳", width=3, command=self.refresh_ports).pack(side="left")
        ttk.Label(bar, text="  Baud").pack(side="left")
        self.baud_var = tk.StringVar(value=str(ARGS.baud))
        baud_box = ttk.Combobox(bar, textvariable=self.baud_var, values=BAUD_RATES, width=9)
        baud_box.pack(side="left", padx=4)
        baud_box.bind("<<ComboboxSelected>>", self.on_baud)
        baud_box.bind("<Return>", self.on_baud)
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=8)
        ttk.Button(bar, text="Clear", command=self.clear).pack(side="left")

        self.text = tk.Text(root, wrap="char", font=("Consolas", 10), bg="#1e1e1e", fg="#d4d4d4",
                            insertbackground="#d4d4d4")
        self.text.tag_config("you", foreground="#4fc1ff")
        self.text.tag_config("claude", foreground="#d7a0ff")
        self.text.tag_config("info", foreground="#888888")
        scroll = ttk.Scrollbar(root, command=self.text.yview)
        self.text.configure(yscrollcommand=scroll.set, state="disabled")

        entry_row = ttk.Frame(root, padding=6)
        entry_row.pack(side="bottom", fill="x")
        self.status = ttk.Label(root, text="", anchor="w", padding=(6, 0))
        self.status.pack(side="bottom", fill="x")
        scroll.pack(side="right", fill="y")
        self.text.pack(fill="both", expand=True)

        ttk.Label(entry_row, text="Command:").pack(side="left", padx=(0, 4))
        self.entry = ttk.Entry(entry_row, font=("Consolas", 12))
        self.entry.pack(side="left", fill="x", expand=True, ipady=3)
        # Typing while the output pane has focus (CoolTerm habit) goes to the entry.
        self.text.bind("<Key>", self.redirect_key)
        self.text.bind("<ButtonRelease-1>", lambda e: self.entry.focus_set())
        self.entry.bind("<Return>", self.on_send)
        self.entry.bind("<Up>", lambda e: self.recall(-1))
        self.entry.bind("<Down>", lambda e: self.recall(1))
        ttk.Button(entry_row, text="Send", command=self.on_send).pack(side="left", padx=4)
        ttk.Button(entry_row, text="Stop (X)", command=lambda: self.link.send("X", "you")).pack(side="left")

        try:
            start_link_server(self.link)
            self.link_text = f"Claude link: {LINK_HOST}:{LINK_PORT}"
        except OSError as e:
            self.link_text = f"Claude link FAILED ({e}) - is another Serial Terminal open?"

        self.refresh_ports(auto_connect=True)
        self.entry.focus_set()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(20, self.pump)

    # --- UI actions ---
    def refresh_ports(self, auto_connect=False):
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        self.port_map = {}
        labels, pico_label, requested_label = [], None, None
        for p in ports:
            label = f"{p.device} - {p.description}"
            if p.vid == PICO_VID:
                label += "  [Pico]"
                pico_label = label
            if ARGS.port and p.device.upper() == ARGS.port.upper():
                requested_label = label
            self.port_map[label] = p.device
            labels.append(label)
        if ARGS.port and requested_label is None:
            requested_label = f"{ARGS.port} - (not present yet)"
            self.port_map[requested_label] = ARGS.port
            labels.insert(0, requested_label)
        self.port_box["values"] = labels
        preferred = requested_label or pico_label
        if self.port_var.get() not in self.port_map:
            self.port_var.set(preferred or (labels[0] if labels else ""))
        if auto_connect and preferred:
            self.toggle_connect()

    def toggle_connect(self):
        if self.link.want_connected:
            self.link.disconnect()
            self.connect_btn.config(text="Connect")
            return
        port = self.port_map.get(self.port_var.get())
        if not port:
            self.set_status("pick a port first")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self.set_status("baud must be a number")
            return
        self.link.connect(port, baud)
        self.connect_btn.config(text="Disconnect")
        self.info(f"[connecting {port} @ {baud}]")

    def on_baud(self, _event=None):
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            return
        self.link.set_baud(baud)
        self.info(f"[baud -> {baud}]")

    def on_send(self, _event=None):
        text = self.entry.get()
        self.entry.delete(0, "end")
        if text:
            self.history.append(text)
        self.history_pos = len(self.history)
        self.link.send(text, "you")

    def redirect_key(self, event):
        if event.keysym in ("Return", "KP_Enter"):
            self.on_send()
        elif event.keysym == "BackSpace":
            self.entry.delete(len(self.entry.get()) - 1, "end")
        elif event.char and event.char.isprintable():
            self.entry.insert("end", event.char)
        else:
            return None  # let Ctrl+C copy, arrows, etc. behave normally
        self.entry.focus_set()
        self.entry.icursor("end")
        return "break"

    def recall(self, step):
        if not self.history:
            return "break"
        self.history_pos = max(0, min(len(self.history), self.history_pos + step))
        self.entry.delete(0, "end")
        if self.history_pos < len(self.history):
            self.entry.insert(0, self.history[self.history_pos])
        return "break"

    def clear(self):
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.configure(state="disabled")

    def on_close(self):
        self.link.disconnect()
        self.log.write(f"\r\n=== session end {datetime.datetime.now().isoformat()} ===\r\n")
        self.root.destroy()

    # --- output ---
    def append(self, text, tag=None):
        at_bottom = self.text.yview()[1] > 0.999
        self.text.configure(state="normal")
        self.text.insert("end", text, tag)
        self.text.configure(state="disabled")
        if at_bottom:
            self.text.see("end")

    def info(self, msg):
        self.append(f"\n{msg}\n", "info")
        self.log.write(f"\r\n{msg}\r\n")

    def set_status(self, msg):
        self.status.config(text=f"{msg}    |    {self.link_text}    |    log: tools/logs/latest.log")

    def pump(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "rx":
                    # Log file keeps every line; the window drops the [DIAG] ones.
                    shown = self.diag.feed(payload)
                    if shown:
                        self.append(shown.replace("\r\n", "\n"))
                    self.log.write(payload)
                elif kind == "sent":
                    text, source, ok = payload
                    mark = "" if ok else "  (NOT SENT - not connected)"
                    self.append(f"\n>>> {text}{mark}\n", source)
                    self.log.write(f"\r\n>>> [{source}] {text}{mark}\r\n")
                elif kind == "status":
                    self.set_status(payload)
                    self.info(f"[{payload}]")
        except queue.Empty:
            pass
        tail = self.diag.flush_idle()
        if tail:
            self.append(tail.replace("\r\n", "\n"))
        self.root.after(20, self.pump)


def _self_test():
    """Asserts over the chunk-splitting cases the filter has to survive."""
    f = DiagFilter()
    assert f.feed("hello\r\n") == "hello\r\n"
    assert f.feed("[DIAG] chip_reset=0x100\r\n") == ""

    # The prompt has no newline: held by feed, released once the stream is quiet.
    assert f.feed("> ") == ""
    assert f.flush_idle() == "> "
    assert f.flush_idle() == ""

    # Tag split across chunks must stay hidden, and stay held while ambiguous.
    split = DiagFilter()
    assert split.feed("[DI") == ""
    assert split.flush_idle() == ""
    assert split.feed("AG] x\r\n") == ""

    # An ordinary line split across chunks still shows, in order.
    partial = DiagFilter()
    assert partial.feed("SEARCH TUR") == ""
    assert partial.feed("N target=E\r\n") == "SEARCH TURN target=E\r\n"

    # One chunk holding both kinds.
    mixed = DiagFilter()
    assert mixed.feed("a\r\n[DIAG] b\r\nc\r\n") == "a\r\nc\r\n"

    print("serial_term self-test: ok")


def main():
    if ARGS.self_test:
        _self_test()
        return

    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
