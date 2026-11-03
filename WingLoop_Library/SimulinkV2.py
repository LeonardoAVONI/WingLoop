# =============================================================================
# WingLoop — Simulink V2 bridge
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Simulink V2 method: Matteo Alfonso Incarbone (lpmatteo241@gmail.com)
#
# This file is part of WingLoop.
# WingLoop is licensed under CC BY-NC-SA 4.0 (Non-Commercial use only).
# Full license: https://creativecommons.org/licenses/by-nc-sa/4.0/
# =============================================================================
"""
Simulink V2: one continuous Simulink simulation, exchanging data with WingLoop over TCP.

Simulink V1 calls sim() once per WingLoop step (slow: ~10x ASWING). V2 (method by
M. A. Incarbone) runs the controller model in a single sim() for the whole run; the
ASWING plant is a MATLAB System block (simulink_v2/AswingPlant.m) that, at every
Simulink step, receives the ASWING state from WingLoop and returns the controller
command.

The user writes the same model as for Simulink V1 / FMU (one Inport "statein",
one Outport per command named F1..F20 / E1..E20). WingLoop builds a wrapper copy
"<model>_wlv2.slx" in which "statein" is driven by AswingPlant and the Outports
feed AswingPlant (simulink_v2/wingloop_v2_make_model.m).

Timing: AswingPlant has no direct feedthrough. At Simulink time t_k its output
(outputImpl) is the ASWING state y_k sent by WingLoop, the controller computes u_k
from it, and updateImpl sends u_k back. So u_k = C(y_k), exactly as for the Python,
MATLAB, Simulink V1 and FMU backends, without an algebraic loop.

Protocol (TCP, little-endian; WingLoop is the server, AswingPlant the client):
    client -> server  hello : uint32 0x574C5632 ("WLV2"), uint32 n_cmd,
                              uint32 n_bytes, n_bytes of UTF-8 comma-separated names
    server -> client  step  : uint32 1, float64 t, float64[n_state] state
    client -> server  cmd   : uint32 2, float64[n_cmd] u
    server -> client  stop  : uint32 0   (AswingPlant then stops the simulation)

Launch: WingLoop writes a handshake JSON (port, sizes, Dt, stop time, model) and
    launch="engine"   (Linux): calls wingloop_v2_run(handshake) in its MATLAB engine,
                      in the background;
    launch="external" (Windows + WSL, untested): MATLAB is started by the user and runs
                      wingloop_v2_windows_launcher.m / wingloop_v2_run(handshake) itself.
"""

import os
import json
import time
import socket
import struct

import numpy as np

MAGIC = 0x574C5632
MSG_STOP, MSG_STEP, MSG_CMD = 0, 1, 2

LIB_DIR = os.path.dirname(os.path.abspath(__file__))
MATLAB_DIR = os.path.join(LIB_DIR, "simulink_v2")


def _recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Simulink closed the connection")
        buf += chunk
    return bytes(buf)


class SimulinkV2Controller:
    """
    Python side of Simulink V2.

    Parameters
    ----------
    control_directory : str     folder of the user's .slx (and UserController.m)
    model_name : str            model name (file without .slx)
    Dt : float                  controller step [s] (WingLoop's Dt*K)
    eng : MATLAB engine | None  required for launch="engine"
    launch : "engine" | "external"
    host, port : listening address (port 0 = any free port; "external" binds 0.0.0.0)
    handshake_file : str | None where the handshake JSON is written
                                (default <control_directory>/wingloop_v2_handshake.json)
    connect_timeout, step_timeout : seconds
    run_user_controller : bool  external mode: let MATLAB instantiate UserController.m
    """

    def __init__(self, control_directory, model_name, Dt, eng=None, launch="engine",
                 host=None, port=0, handshake_file=None, connect_timeout=300.0,
                 step_timeout=1200.0, show_simulink=False, run_user_controller=None,
                 precomputed_path=""):
        if launch not in ("engine", "external"):
            raise ValueError("launch must be 'engine' or 'external'")
        if launch == "engine" and eng is None:
            raise ValueError("launch='engine' needs a MATLAB engine")
        self.control_directory = os.path.abspath(control_directory)
        self.model_name = model_name
        self.Dt = float(Dt)
        self.eng = eng
        self.launch = launch
        self.host = host or ("127.0.0.1" if launch == "engine" else "0.0.0.0")
        self.port = int(port)
        self.handshake_file = handshake_file or os.path.join(self.control_directory,
                                                             "wingloop_v2_handshake.json")
        self.connect_timeout = float(connect_timeout)
        self.step_timeout = float(step_timeout)
        self.show_simulink = show_simulink
        self.run_user_controller = (launch == "external") if run_user_controller is None else run_user_controller
        self.precomputed_path = precomputed_path or ""
        self.n_calls = None            # number of controller steps of the run (sets StopTime)
        self.n_state = None
        self.names = None
        self.time = 0.0
        self._srv = self._conn = self._future = None
        self.startup_time = None      # [s] handshake -> Simulink connected (model build + compile)
        self.wait_time = 0.0          # [s] total time waiting for Simulink's commands
        self.n_steps = 0

    # ------------------------------------------------------------------
    def prepare_run(self, n_calls):
        """Number of controller steps the run will make (WingLoop calls this before the march)."""
        self.n_calls = int(n_calls)

    def _start(self, n_state):
        t_begin = time.time()
        self.n_state = int(n_state)
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        stop_time = (self.n_calls - 1) * self.Dt if self.n_calls else 1e9
        hs = {
            "port": self.port,
            "host": "127.0.0.1" if self.host in ("0.0.0.0", "") else self.host,
            "n_state": self.n_state,
            "Dt": self.Dt,
            "stop_time": stop_time,
            "model": self.model_name,
            "model_dir": self.control_directory,
            "matlab_dir": MATLAB_DIR,
            "show_simulink": bool(self.show_simulink),
            "run_user_controller": bool(self.run_user_controller),
            "precomputed": self.precomputed_path,
            "timeout": self.step_timeout,
        }
        tmp = self.handshake_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump(hs, f, indent=2)
        os.replace(tmp, self.handshake_file)          # atomic: the launcher never reads half a file
        print(f"[SimulinkV2] listening on {self.host}:{self.port}; handshake {self.handshake_file}")

        if self.launch == "engine":
            self.eng.addpath(MATLAB_DIR, nargout=0)
            self._future = self.eng.wingloop_v2_run(self.handshake_file, nargout=0, background=True)
        else:
            print("[SimulinkV2] waiting for MATLAB: run wingloop_v2_run(<handshake>) "
                  "(or wingloop_v2_windows_launcher) on the MATLAB side")

        self._srv.settimeout(1.0)
        t0 = time.time()
        while True:
            try:
                self._conn, addr = self._srv.accept()
                break
            except socket.timeout:
                if self._future is not None and self._future.done():
                    self._future.result()            # raises the MATLAB error
                    raise RuntimeError("Simulink V2 run ended before connecting")
                if time.time() - t0 > self.connect_timeout:
                    raise TimeoutError(f"Simulink V2 did not connect within {self.connect_timeout:g} s")
        self._conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._conn.settimeout(self.step_timeout)
        magic, n_cmd, nb = struct.unpack("<III", _recv_exact(self._conn, 12))
        if magic != MAGIC:
            raise RuntimeError(f"Simulink V2: bad hello (magic {magic:#x})")
        self.names = [n.strip() for n in _recv_exact(self._conn, nb).decode().split(",") if n.strip()]
        if len(self.names) != n_cmd:
            raise RuntimeError(f"Simulink V2: {n_cmd} commands announced, names {self.names}")
        self.startup_time = time.time() - t_begin
        print(f"[SimulinkV2] Simulink connected from {addr} after {self.startup_time:.1f} s; commands {self.names}")

    # ------------------------------------------------------------------
    def step(self, instantaneous_state, Dt=None):
        """Send y_k, return {name: u_k}."""
        state = np.ascontiguousarray(np.asarray(instantaneous_state, dtype="<f8").ravel())
        if self._conn is None:
            self._start(state.size)
        if state.size != self.n_state:
            raise ValueError(f"state size {state.size} != {self.n_state} announced to Simulink")
        t0 = time.time()
        self._conn.sendall(struct.pack("<Id", MSG_STEP, self.time) + state.tobytes())
        try:
            (kind,) = struct.unpack("<I", _recv_exact(self._conn, 4))
            if kind != MSG_CMD:
                raise RuntimeError(f"Simulink V2: unexpected message {kind}")
            u = np.frombuffer(_recv_exact(self._conn, 8 * len(self.names)), dtype="<f8")
        except (ConnectionError, socket.timeout) as exc:
            err = None
            if self._future is not None:
                try:
                    self._future.result(timeout=30)       # the MATLAB-side cause, if it ended
                except Exception as e:
                    err = e
            raise RuntimeError(f"Simulink V2 step at t={self.time:g} failed: {exc}"
                               + (f"; MATLAB: {err}" if err else "")) from exc
        self.wait_time += time.time() - t0
        self.n_steps += 1
        self.time += self.Dt if Dt is None else Dt
        return dict(zip(self.names, (float(v) for v in u)))

    # ------------------------------------------------------------------
    def terminate(self):
        """Stop the Simulink run (if still running) and close the sockets."""
        if self.n_steps:
            print(f"[SimulinkV2] start-up {self.startup_time:.1f} s, Simulink step "
                  f"{self.wait_time / self.n_steps * 1000:.2f} ms on average ({self.n_steps} steps)")
        if self._conn is not None:
            try:
                self._conn.sendall(struct.pack("<I", MSG_STOP))
            except OSError:
                pass
        if self._future is not None:
            try:
                self._future.result(timeout=60)
            except Exception as exc:
                if "WingLoop:stop" not in str(exc) and "stopped by WingLoop" not in str(exc):
                    print(f"[SimulinkV2] MATLAB run ended with: {exc}")
        for s in (self._conn, self._srv):
            try:
                if s is not None:
                    s.close()
            except OSError:
                pass
        self._conn = self._srv = self._future = None
