from sys import argv
from subprocess import Popen, PIPE, DEVNULL
from threading import Thread
from re import compile

def monitor_ffplay(state, proc):
    re_time_seconds = compile(rb"\b([0-9]+(?:\.[0-9]+)?)\s+A-V:")
    buf = b""
    while True:
        chunk = proc.stderr.read(1)
        if not chunk:
            break
        buf += chunk
        # Keep buffer from growing indefinitely
        if len(buf) > 65536:
            buf = buf[-65536:]
        # Search for the last match in the buffer (handles in-place \r updates)
        matches = list(re_time_seconds.finditer(buf))
        if matches:
            m = matches[-1]
            state["time"] = float(m.group(1).decode())
        # Also remove everything up to the last carriage return/newline to avoid reparsing stale data
        last_delim = max(buf.rfind(b"\r"), buf.rfind(b"\n"))
        if last_delim != -1:
            buf = buf[last_delim + 1:]
    print("monitor_ffplay done")

def input_loop(state):
    path = argv[2]
    print(f"Saving output to {path}")

    prompt = "Press Enter to save current time: "
    while True:
        try:
            input(prompt)
        except EOFError:
            break
        t = state["time"]
        if t is None:
            print("Time: unknown (no timestamp parsed yet)")
        else:
            line = f"{t}\n"
            with open(path, "a", encoding="utf-8") as f:
                f.write(line)
            print(f"Saved: {t}")
    print("input_loop done")

if __name__ == "__main__":
    print("Usage: python3 ball_counter.py <video> <output>")

    video = argv[1]
    print(f"Playing {video}")
    proc = Popen(
        # -sync video ensures that timestamps match with video, not audio
        ["ffplay", "-sync", "video", "-autoexit", video],
        stderr=PIPE,
        stdout=DEVNULL
    )

    state = {"time": None}

    Thread(target=monitor_ffplay, args=(state, proc), daemon=True).start()
    input_loop(state)
