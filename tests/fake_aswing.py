#!/usr/bin/env python3
"""
Minimal stand-in for the ASWING executable, for WingLoop's tests (no ASWING needed).

It prints ASWING's prompts (no newline after "c>  "), answers the commands WingLoop
sends (PLPA/G, gget/pget/sget/tget/cget, OPER, ".", k-menu flags, "x [file]" + "Dt n",
"W file", "-", "A alt g", HSAV, QUIT), and writes point files built from a real ASWING
point file (tests/data/point_template). In every point file:

    Time      : the simulated time
    Status    : Converged / Not converged (see FAKE_NC)
    Flap 2    : the F2 value read from the control file for that step (0 without file)
    state[0]  : the time-level index (0 = initial point), to detect lost/duplicated samples
    state[1]  : the time

Behaviour switches (environment variables):
    FAKE_NC="4,7"       time levels whose first solve does not converge (a retry converges)
    FAKE_NC_HARD="0.09" times [s] at which no solve ever converges (also after a retry)
    FAKE_HANG_LEVEL=5   stop answering when marching to this level
    FAKE_CRASH_LEVEL=5  exit when marching to this level
"""
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = open(os.path.join(HERE, "data", "point_template")).read()


def env_set(name):
    return {int(x) for x in os.environ.get(name, "").split(",") if x.strip()}


NC = env_set("FAKE_NC")
NC_HARD = [float(x) for x in os.environ.get("FAKE_NC_HARD", "").split(",") if x.strip()]
HANG = int(os.environ.get("FAKE_HANG_LEVEL", "-1"))
CRASH = int(os.environ.get("FAKE_CRASH_LEVEL", "-1"))

out = sys.stdout


def say(text):
    out.write(text)
    out.flush()


def prompt(menu):
    say("\n " + menu + "   c>  ")


def read_controls(path):
    """{name: [(t, value), ...]} from an ASWING time-history control file."""
    with open(path) as f:
        lines = [ln.split() for ln in f if ln.strip()]
    names = lines[0][1:]
    rows = [[float(x) for x in ln] for ln in lines[1:]]
    return {n: [(r[0], r[i + 1]) for r in rows] for i, n in enumerate(names)}


def value_at(trace, t):
    # nearest row in time (WingLoop writes either 2 constant rows or one row per step end time)
    return min(trace, key=lambda r: abs(r[0] - t))[1] if len(trace) > 2 else trace[0][1]


class Fake:
    def __init__(self):
        self.menu = "main"
        self.unsteady = False
        self.points = [{"level": 0, "time": 0.0, "conv": False, "F2": 0.0}]   # loaded trim
        self.attempts = {}
        self.flags = {"F": True, "L": True}

    def current(self):
        return self.points[-1]

    def write_point(self, fname):
        p = self.current()
        txt = TEMPLATE
        txt = re.sub(r"Time    :\s*\S+", f"Time    :  {p['time']:.4f}", txt, count=1)
        txt = re.sub(r"Status: [^\n]*", "Status: " + ("Converged" if p["conv"] else "Not converged"), txt, count=1)
        txt = re.sub(r"Flap 2 :\s*\S+", f"Flap 2 : {p['F2']:.6g}", txt, count=1)
        head, _, rest = txt.partition("STARTEDPRINTINGSTATES\n")
        body = rest.split("\n")
        body[0] = f"  {float(p['level']):.8E}"
        body[1] = f"  {p['time']:.8E}"
        with open(fname, "w") as f:
            f.write(head + "STARTEDPRINTINGSTATES\n" + "\n".join(body))

    def march(self, dt, n, control):
        traces = read_controls(control) if control else None
        if n > 0:                                   # new march from the current (initial) point
            del self.points[1:]
        for _ in range(abs(n)):
            last = self.current()
            level = last["level"] + 1
            t = last["time"] + dt
            if level == HANG:
                time.sleep(3600)
            if level == CRASH:
                sys.exit(3)
            k = self.attempts.get(level, 0)
            self.attempts[level] = k + 1
            hard = any(abs(t - th) < 1e-9 for th in NC_HARD)
            conv = not (hard or (level in NC and k == 0))
            f2 = value_at(traces["F2"], t) if traces and "F2" in traces else 0.0
            say(f"\n Converging Point {level} ... t = {t}\n")
            if not conv:
                say("   Newton restart #           1\n")
            self.points.append({"level": level, "time": t, "conv": conv, "F2": f2})
            say(" ......................\n")

    def run(self):
        prompt("ASWING")
        pending_march = None
        for raw in sys.stdin:
            line = raw.rstrip("\n")
            cmd, _, arg = line.strip().partition(" ")
            c = cmd.upper()
            if pending_march is not None:
                dt, n = line.split()
                self.march(float(dt), int(n), pending_march or None)
                pending_march = None
                prompt(".OPERu (initial point: %3d)" % self.current()["level"])
                continue
            if self.menu == "main":
                if c == "QUIT":
                    return
                if c == "OPER":
                    self.menu = "oper"
                    prompt(".OPERs (target point:  1)" if not self.unsteady else ".OPERu (initial point:   1)")
                    continue
                if c == "HSAV":
                    open(arg.strip(), "wb").write(b"fake state")
                prompt("ASWING")
                continue
            # OPER menu
            if c == "":
                self.menu = "main"
                prompt("ASWING")
            elif c == ".":
                self.unsteady = not self.unsteady
                prompt(".OPERu (initial point:   1)" if self.unsteady else ".OPERs (target point:  1)")
            elif c == "K":
                self.menu_k()
            elif c == "%":
                self.menu_table()
            elif c == "X" and self.unsteady:
                pending_march = arg.strip()
                say("\n Enter delta(t), Nstep  (-Nstep to append steps)  0.10000      10\n")
            elif c == "W":
                self.write_point(arg.strip())
                prompt(".OPERu (initial point: %3d)" % self.current()["level"])
            elif c == "-":
                if len(self.points) > 1:
                    self.points.pop()
                prompt(".OPERu (initial point: %3d)" % self.current()["level"])
            else:
                prompt(".OPERu (initial point: %3d)" % self.current()["level"])

    def menu_table(self):
        """Constraint table: records the keys (written to table_keys.log), "d" or blank leaves it."""
        def ask():
            say(" Type indices of (col,row) to toggle, or ( ABCDFLPQSX.+- ): ")
        ask()
        for raw in sys.stdin:
            k = raw.strip()
            if k.upper() in ("D", ""):
                prompt(".OPERu (initial point:   1)" if self.unsteady else ".OPERs (target point:  1)")
                return
            with open("table_keys.log", "a") as f:
                f.write(k + "\n")
            if k.upper() == "C":
                self.unsteady = True          # ptable.f: "c" sets STEADY = F
            ask()

    def menu_k(self):
        def show():
            say(f"\n F reeze VL matrices                     :  {'T' if self.flags['F'] else 'F'}\n")
            say(f" L ag aero effects modeled                :  {'T' if self.flags['L'] else 'F'}\n")
            say("\n Select option   c>  ")
        show()
        for raw in sys.stdin:
            c = raw.strip().upper()
            if c == "":
                prompt(".OPERu (initial point:   1)" if self.unsteady else ".OPERs (target point:  1)")
                return
            if c in self.flags:
                self.flags[c] = not self.flags[c]
            show()


if __name__ == "__main__":
    Fake().run()
