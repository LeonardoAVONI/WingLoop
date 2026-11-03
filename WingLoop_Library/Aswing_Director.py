# =============================================================================
# WingLoop — Aswing Director
# =============================================================================
# Copyright (c) 2024-2026 Leonardo Avoni (avonileonardo@gmail.com)
#
# This file is part of WingLoop.
# WingLoop is licensed under CC BY-NC-SA 4.0 (Non-Commercial use only).
# Full license: https://creativecommons.org/licenses/by-nc-sa/4.0/
#
# For commercial use, contact the author: avonileonardo@gmail.com
#
# If you use WingLoop in academic work, please cite:
#   Avoni et al., "Enhancing ASWING Flight Dynamics Simulations with
#   Closed-Loop Control for Flexible Aircraft," AIAA 2025-3425.
#   https://arc.aiaa.org/doi/10.2514/6.2025-3425
# =============================================================================

"""
====================================================================================
Aswing Director — WingLoop 3.0
====================================================================================

Drives the Fortran ASWING executable as a subprocess (stdin/stdout).

Synchronisation (V3)
--------------------
ASWING's menu prompts end with "   c>  ", "   s>  ", "   i>  ", "   r>  " or
" y/n>  " and *no newline*. ASWING is started with
GFORTRAN_UNBUFFERED_PRECONNECTED=y, so a prompt reaches Python as soon as ASWING
prints it, and a reader thread collects the raw output (not line by line).

    sync="prompt" (default)
        send_command_and_receive() sends a command and returns once ASWING shows
        its next prompt, i.e. once the command is finished. A file ASWING writes
        is closed before the next prompt appears, so it is complete when the call
        returns. Questions printed as a plain line (for example "Enter delta(t),
        Nstep") are not prompts: pass `expect=` (a regex matched on the end of the
        output) for those. Without `expect`, a command that prints no prompt
        returns after `quiet_time` seconds without output (a fallback for
        arbitrary user commands; WingLoop itself always passes `expect`).
    sync="timer"
        V2 behaviour: send, then collect output for a fixed time (custom_timer or
        wait_time). Kept for scripts that depend on it.

Every prompt-synchronised command has a timeout (`command_timeout`, `write_timeout`
for marches and file writes). On a timeout the ASWING process group is killed and
ASWINGHangError is raised; if ASWING exits, ASWINGCrashError is raised.

Compatibility: `stdout_queue` / `stderr_queue` still receive ASWING's output (bounded
to the last `queue_maxlen` chunks), so code that reads them keeps working.

Executable: `aswing_path` may be the executable itself or its folder; the shell alias
(`aswing_alias`, resolved through `bash -ic 'type <alias>'`) is only a fallback. Resolve
the alias once in the main process: concurrent `bash -ic` shells in worker processes
can be stopped by SIGTTOU.

Changelog: V4 (21/10/2024) … V9 (17/12/2024), package (29/05/2025), WingLoop 2.0
(10/03/2026); WingLoop 3.0 (11/2026): prompt synchronisation, timeouts and hang
handling, single-threaded BLAS in the ASWING process, executable path or folder.
====================================================================================
"""

import os
import re
import sys
import time
import signal
import threading
import subprocess
from collections import deque
from queue import Empty

__all__ = ["Aswing_Director", "ASWINGHangError", "ASWINGCrashError", "PROMPT_RE", "TABLE_PROMPT_RE",
           "resolve_aswing_executable"]

# end of an ASKC/ASKS/ASKI/ASKR/ASKL prompt (ask1.f: FORMAT(/A,'   c>  ',$) etc.)
PROMPT_RE = re.compile(r"(?:\s[cisr]|\sy/n)>  $")
# plain-line question of the unsteady X command (oper.f, FORMAT 2052)
MARCH_QUESTION_RE = re.compile(r"append steps\)[^\n]*\n\s*$")
# prompt of the OPER constraint table ("%", ptable.f), printed without newline
TABLE_PROMPT_RE = re.compile(r"\( ABCDFLPQSX\.\+- \): $")
# a question ASWING asks before overwriting an existing file
OVERWRITE_QUESTION_RE = re.compile(r"(exists\.[^\n]*(Overwrite|Append)[^\n]*\n\s*$)|(Overwrite\?\s*Y\s*\n\s*$)")


class ASWINGHangError(RuntimeError):
    """ASWING did not answer a command within its timeout (the process was killed)."""


class ASWINGCrashError(RuntimeError):
    """The ASWING process exited while a command was running."""


def resolve_aswing_executable(aswing_path=None, aswing_alias=None):
    """Return the absolute path of the ASWING executable.

    aswing_path: the executable, or the folder that contains an executable named "aswing".
    aswing_alias: shell alias used when aswing_path is None (fallback; slow, call once).
    """
    if aswing_path:
        p = os.path.abspath(os.path.expanduser(str(aswing_path)))
        if os.path.isdir(p):
            p = os.path.join(p, "aswing")
        if not (os.path.isfile(p) and os.access(p, os.X_OK)):
            raise FileNotFoundError(f"ASWING executable not found or not executable: {p}")
        return p
    if not aswing_alias:
        raise ValueError("Give aswing_path (executable or folder) or aswing_alias.")
    try:
        out = subprocess.check_output(f"bash -ic 'type {aswing_alias}'", shell=True, text=True,
                                      stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL).strip()
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Could not resolve ASWING alias '{aswing_alias}' (check ~/.bashrc).") from exc
    m = re.search(r"`([^']+)'", out) or re.search(r"is (/\S+)$", out)
    if not m:
        raise RuntimeError(f"Could not parse an executable path from: {out!r}")
    return resolve_aswing_executable(m.group(1))


class _TailQueue:
    """Bounded stand-in for queue.Queue (put/get/get_nowait/empty); drops the oldest items."""

    def __init__(self, maxlen):
        self._d = deque(maxlen=maxlen)
        self._cv = threading.Condition()

    def put(self, item):
        with self._cv:
            self._d.append(item)
            self._cv.notify_all()

    def get_nowait(self):
        with self._cv:
            if not self._d:
                raise Empty
            return self._d.popleft()

    def get(self, block=True, timeout=None):
        if not block:
            return self.get_nowait()
        with self._cv:
            if not self._cv.wait_for(lambda: bool(self._d), timeout=timeout):
                raise Empty
            return self._d.popleft()

    def empty(self):
        with self._cv:
            return not self._d

    def qsize(self):
        with self._cv:
            return len(self._d)


class Aswing_Director:
    """
    Interface to one ASWING process.

    Parameters
    ----------
    aswing_path : str | None
        ASWING executable, or the folder containing it. If None, `aswing_alias` is resolved.
    aswing_alias : str | None
        Shell alias of ASWING (fallback).
    wait_time : float
        sync="timer": collection time per command [s]. sync="prompt": unused, except as
        the start-up wait in timer mode.
    finished_writing_file_check_timestep : float
        Polling interval [s] for the (fallback) file-completion checks. Default 2 ms.
    write_timeout : float
        Timeout [s] of time marches and file writes. A hard time step (many Newton restarts
        and sub-steps) can take 40-45 s alone and > 100 s under parallel load. Default 1200 s.
    command_timeout : float
        Timeout [s] of other commands (trims can be long). Default: write_timeout.
    sync : "prompt" | "timer"
    quiet_time : float
        sync="prompt": a command that prints no prompt and matches no `expect` returns after
        this many seconds without new output.
    single_thread_blas : bool
        Set OMP/OPENBLAS/MKL thread counts to 1 in ASWING's environment (parallel runs).
    """

    def __init__(self, aswing_path=None, aswing_alias=None,
                 wait_time=0.03,
                 finished_writing_file_check_timestep=0.002,
                 write_timeout=1200.0,
                 command_timeout=None,
                 sync="prompt",
                 quiet_time=2.0,
                 single_thread_blas=True,
                 queue_maxlen=5000):
        if sync not in ("prompt", "timer"):
            raise ValueError("sync must be 'prompt' or 'timer'")
        self.aswing_exe = resolve_aswing_executable(aswing_path, aswing_alias)
        self.aswing_path = os.path.dirname(self.aswing_exe)   # V2 attribute (a folder)
        self.asw_process = None
        self.wait_time = wait_time
        self.finished_writing_file_check_timestep = finished_writing_file_check_timestep
        self.write_timeout = float(write_timeout)
        self.command_timeout = float(command_timeout) if command_timeout is not None else None
        self.sync = sync
        self.quiet_time = float(quiet_time)
        self.single_thread_blas = single_thread_blas
        self.print_output = False
        self.stdout_queue = _TailQueue(queue_maxlen)
        self.stderr_queue = _TailQueue(queue_maxlen)
        self._buf = []                 # chunks not yet returned by a prompt-synchronised command
        self._cv = threading.Condition()
        self._eof = False
        self.last_output = ""
        self.directory = None

    # ------------------------------------------------------------------ process
    def _reader(self, fd, mirror, to_buffer):
        try:
            while True:
                try:
                    chunk = os.read(fd, 65536)
                except OSError:
                    break
                if not chunk:
                    break
                text = chunk.decode("utf-8", errors="replace")
                mirror.put(text)
                if self.print_output:
                    sys.stdout.write(text)
                    sys.stdout.flush()
                with self._cv:
                    if to_buffer or self.sync == "prompt":
                        self._buf.append(text)
                    self._cv.notify_all()
        finally:
            if to_buffer:
                with self._cv:
                    self._eof = True
                    self._cv.notify_all()

    def start_aswing(self, directory=".", filename=None, print_output=True):
        """Start ASWING in `directory` (os.chdir into it: WingLoop uses relative exchange files),
        optionally with a geometry file; waits for the first prompt."""
        self.previous_directory = os.getcwd()
        self.print_output = print_output
        os.chdir(directory)
        self.directory = os.getcwd()
        cmd = [self.aswing_exe]
        if filename:
            cmd.append(filename[:-4] if filename.endswith(".asw") else filename)
        env = dict(os.environ, GFORTRAN_UNBUFFERED_PRECONNECTED="y")
        if self.single_thread_blas:
            for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
                env[k] = "1"
        self.asw_process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE, bufsize=0, env=env,
                                            start_new_session=True)
        threading.Thread(target=self._reader, args=(self.asw_process.stdout.fileno(), self.stdout_queue, True),
                         daemon=True).start()
        threading.Thread(target=self._reader, args=(self.asw_process.stderr.fileno(), self.stderr_queue, False),
                         daemon=True).start()
        if self.sync == "prompt":
            self._wait(None, self._timeout(None), what="start-up")
        else:
            time.sleep(self.wait_time)

    def is_alive(self):
        return self.asw_process is not None and self.asw_process.poll() is None

    def kill(self):
        """Kill the ASWING process group (used on hangs)."""
        if self.asw_process is None:
            return
        try:
            os.killpg(self.asw_process.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                self.asw_process.kill()
            except Exception:
                pass
        try:
            self.asw_process.wait(5)
        except Exception:
            pass

    # ------------------------------------------------------------------ commands
    def _timeout(self, timeout):
        if timeout is not None:
            return float(timeout)
        return self.command_timeout if self.command_timeout is not None else self.write_timeout

    def _wait(self, expect, timeout, what):
        """Wait until the collected output ends with a prompt or matches `expect`; return it."""
        rx = re.compile(expect) if isinstance(expect, str) else expect
        t0 = time.time()
        last_len, last_change = 0, t0
        with self._cv:
            while True:
                text = "".join(self._buf)
                tail = text[-600:]
                if PROMPT_RE.search(tail) or (rx is not None and rx.search(tail)):
                    self._buf.clear()
                    self.last_output = text
                    return text
                now = time.time()
                if len(text) != last_len:
                    last_len, last_change = len(text), now
                if self._eof:
                    self._buf.clear()
                    raise ASWINGCrashError(f"ASWING exited during {what!r}. Last output:\n{text[-1500:]}")
                if rx is None and now - last_change > self.quiet_time and text.endswith("\n"):
                    self._buf.clear()
                    self.last_output = text
                    return text
                if now - t0 > timeout:
                    self._buf.clear()
                    self.kill()
                    raise ASWINGHangError(f"ASWING gave no answer to {what!r} within {timeout:g} s "
                                          f"(process killed). Last output:\n{text[-1500:]}")
                self._cv.wait(timeout=0.05 if rx is None else 0.5)

    def _send(self, command):
        if not self.is_alive():
            raise ASWINGCrashError(f"ASWING is not running (command {command!r}).")
        try:
            self.asw_process.stdin.write((command + "\n").encode())
            self.asw_process.stdin.flush()
        except BrokenPipeError as exc:
            raise ASWINGCrashError(f"ASWING closed its input (command {command!r}).") from exc

    def send_command_and_receive(self, command, custom_timer=None, expect=None, timeout=None):
        """Send one command; return (stdout, stderr).

        sync="prompt": returns once ASWING shows its next prompt or the output matches
        `expect`; `custom_timer` is ignored. sync="timer": V2 behaviour.
        """
        if self.sync == "timer":
            timer = self.wait_time if custom_timer is None else custom_timer
            out, err = [], []
            self._send(command)
            t0 = time.time()
            while time.time() - t0 < timer:
                for q, acc in ((self.stdout_queue, out), (self.stderr_queue, err)):
                    try:
                        acc.append(q.get_nowait())
                    except Empty:
                        pass
            with self._cv:
                self._buf.clear()
            return "".join(out), "".join(err)
        with self._cv:
            self._buf.clear()
        self._send(command)
        return self._wait(expect, self._timeout(timeout), what=command), ""

    def send_writefile_command_and_receive(self, filename, custom_timer=None, append_or_overwrite=None,
                                           end_marker=None, timeout=None):
        """Answer a file-name question with `filename` (the write command was sent before),
        answer an overwrite question if one comes, and wait until the file is complete.

        Returns (stdout, stderr, seconds). In prompt mode the file is complete when ASWING
        prompts again; `end_marker` is then checked once. Raises if the file is missing or
        incomplete.
        """
        t0 = time.time()
        tmo = self.write_timeout if timeout is None else timeout
        if self.sync == "prompt":
            out, err = self.send_command_and_receive(filename, expect=OVERWRITE_QUESTION_RE, timeout=tmo)
            if OVERWRITE_QUESTION_RE.search(out[-600:]):
                more, _ = self.send_command_and_receive(append_or_overwrite or "O", timeout=tmo)
                out += more
            if not os.path.exists(filename):
                raise RuntimeError(f"ASWING did not write {filename!r} (cwd {os.getcwd()}). Output:\n{out[-1500:]}")
            if end_marker is not None and not self._ends_with(filename, end_marker):
                raise RuntimeError(f"{filename!r} is incomplete (no {end_marker!r}) although ASWING returned.")
            return out, err, time.time() - t0
        # timer mode: V2 polling (now at `finished_writing_file_check_timestep`, not a busy loop)
        out, err = self.send_command_and_receive(filename, custom_timer=custom_timer)
        if append_or_overwrite:
            o2, e2 = self.send_command_and_receive(append_or_overwrite, custom_timer=custom_timer)
            out, err = out + o2, err + e2
        self.wait_for_file(filename, end_marker=end_marker, timeout=tmo)
        return out, err, time.time() - t0

    # V2 name (both write helpers are the same in V3)
    send_writefile_command_and_receive_old = send_writefile_command_and_receive

    def wait_for_file(self, filename, end_marker=None, timeout=None):
        """Poll until `filename` exists, has a stable non-zero size and (optionally) ends with end_marker."""
        timeout = self.write_timeout if timeout is None else timeout
        t0, last = time.time(), -1
        while time.time() - t0 < timeout:
            if os.path.exists(filename):
                size = os.stat(filename).st_size
                if size > 0 and size == last and (end_marker is None or self._ends_with(filename, end_marker)):
                    return
                last = size
            if not self.is_alive():
                raise ASWINGCrashError(f"ASWING exited while writing {filename!r}.")
            time.sleep(self.finished_writing_file_check_timestep)
        self.kill()
        raise ASWINGHangError(f"{filename!r} not complete after {timeout:g} s (ASWING killed).")

    def write_point_file(self, filename, timeout=None, end_marker="ENDEDPRINTINGSTATES"):
        """OPER menu: write the current point to `filename` ("W <file>") and return the time taken.
        The previous file is deleted first, so ASWING asks no overwrite question."""
        t0 = time.time()
        if os.path.exists(filename):
            os.remove(filename)
        if self.sync == "prompt":
            self.send_command_and_receive("W " + filename, timeout=self.write_timeout if timeout is None else timeout)
            if not os.path.exists(filename) or not self._ends_with(filename, end_marker):
                raise RuntimeError(f"Point file {filename!r} missing or incomplete after 'W'. "
                                   f"Output:\n{self.last_output[-1500:]}")
        else:
            self.send_command_and_receive("W")
            self.send_writefile_command_and_receive(filename, end_marker=end_marker, timeout=timeout)
        return time.time() - t0

    def march(self, Dt, nsteps, control_file=None, append=True, timeout=None):
        """OPER (unsteady) menu: march `nsteps` time steps of `Dt` ("x [file]", then "Dt [-]n").
        Returns ASWING's output of the march."""
        x = "x" if not control_file else "x " + control_file
        n = f"-{int(nsteps)}" if append else f"{int(nsteps)}"
        step = f"{float(Dt)!r} {n}"
        tmo = self.write_timeout if timeout is None else timeout
        if self.sync == "prompt":
            o1, _ = self.send_command_and_receive(x, expect=MARCH_QUESTION_RE, timeout=tmo)
            o2, _ = self.send_command_and_receive(step, timeout=tmo)
            return o1 + o2
        o1, _ = self.send_command_and_receive(x)
        o2, _ = self.send_command_and_receive(step)
        return o1 + o2

    @staticmethod
    def _ends_with(filename, marker, tail_bytes=256):
        """True if the last tail_bytes of the file contain marker."""
        try:
            with open(filename, "rb") as fh:
                fh.seek(0, os.SEEK_END)
                fh.seek(max(0, fh.tell() - tail_bytes))
                return marker.encode() in fh.read()
        except OSError:
            return False

    def Load_Files_ASW(self, filename):
        """Main menu: load a file by extension (.asw load, .pnt pget, .set sget, .state tget,
        .gust gget, .con cget: an ASWING-native control law)."""
        cmds = {".asw": "load ", ".pnt": "pget ", ".set": "sget ", ".state": "tget ", ".gust": "gget ",
                ".con": "cget "}
        ext = os.path.splitext(filename)[1].lower()
        if ext not in cmds:
            raise ValueError(f"Unknown file type: {filename}")
        return self.send_command_and_receive(cmds[ext] + filename)

    def quit_and_close_aswing(self):
        """Send QUIT and wait for the process to end (killed after 10 s)."""
        if self.is_alive():
            try:
                self._send("QUIT")
            except ASWINGCrashError:
                pass
            try:
                self.asw_process.wait(10)
            except subprocess.TimeoutExpired:
                self.kill()
        for s in (self.asw_process.stdin,):
            try:
                s.close()
            except Exception:
                pass
        if self.print_output:
            print("ASWING is closed")


if __name__ == "__main__":
    # Tutorial: start ASWING on the test aircraft, open OPER, and quit.
    here = os.path.dirname(os.path.abspath(__file__))
    asw = Aswing_Director(aswing_alias="aswing_stable")
    asw.start_aswing(directory=os.path.join(here, "test_files", "test_aircraft"),
                     filename="wingloop_test_aircraft", print_output=True)
    asw.send_command_and_receive("OPER")
    asw.send_command_and_receive("")
    asw.quit_and_close_aswing()
