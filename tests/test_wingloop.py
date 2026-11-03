"""
WingLoop 3.0 tests that need no ASWING (a fake ASWING, tests/fake_aswing.py, answers the
commands). Run from the repository root:  python -m pytest
"""
import os
import re
import sys
import shutil
import subprocess

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
os.environ["FAKE_PYTHON"] = sys.executable

from WingLoop_Library import WingLoop, import_data_dict, export_data_dict          # noqa: E402
from WingLoop_Library.PyControl_IO import (parse_point_file, _build_pattern, IncompletePointFile,  # noqa: E402
                                           python2text, python2text_schedule, read_aswing_file)
from WingLoop_Library.WingLoop import GLOBAL_VARIABLES, CONTROL_ELEMENTS             # noqa: E402

FAKE = os.path.join(HERE, "fake_bin", "aswing")
DATA = os.path.join(HERE, "data")


# ----------------------------------------------------------------------------- helpers
def pattern():
    req, rm = list(GLOBAL_VARIABLES), {}
    for c in CONTROL_ELEMENTS:
        raw = ("Flap " if c[0] == "F" else "Peng ") + c[1:]
        req.append(raw)
        rm[raw] = c
    from WingLoop_Library.PyControl_IO import initialize_data_dict
    d = initialize_data_dict(req, rm, {})
    return _build_pattern(d, rm), rm, d


@pytest.fixture
def case(tmp_path):
    """A case folder with a dummy geometry and the files WingLoop loads."""
    for f in ("case.asw", "case.pnt", "case.set", "case.state", "case.gust"):
        (tmp_path / f).write_text("dummy\n")
    return tmp_path


def run(case, N=10, K=1, controller=None, env=None, monkeypatch=None, **kw):
    for k, v in (env or {}).items():
        monkeypatch.setenv(k, v)
    WL = WingLoop()
    if controller:
        WL.Launch_WingLoop_Control(os.path.dirname(controller), os.path.basename(controller), timestep=0.01 * K)
    else:
        WL.Launch_WingLoop_Control(None, None, timestep=0.01)
    WL.Launch_ASWING(aswing_fullpath=FAKE, sim_directory=str(case), asw_filename="case.asw",
                     write_timeout=kw.pop("write_timeout", 20), command_timeout=kw.pop("command_timeout", 20))
    WL.Deactivate_Graphics()
    for f in ("case.gust", "case.pnt", "case.set", "case.state"):
        WL.Load_Files(f)
    ok = WL.Time_Transient_Simulation(0.01, N, K, **kw)
    V = WL.WingLoop_LogFile["ModelVariables"]
    S = WL.WingLoop_LogFile["ModelStates"][: WL.WingLoop_LogFile["_state_count"]]
    WL.Closing_WingLoop()
    return ok, WL, V, S


def write_controller(path, body):
    path.write_text("import numpy as np\n\nclass UserController:\n"
                    "    def __init__(self, precomputed_file_path=None):\n        self.calls = []\n" + body)
    return str(path)


# ----------------------------------------------------------------------------- parser / IO
def test_parse_point_file_fields():
    pat, rm, d = pattern()
    p = parse_point_file(os.path.join(DATA, "point_template"), pat, rm, wanted=d["ModelVariables"])
    assert p["converged"] is True
    assert p["values"]["Time"] == pytest.approx(1.49)
    assert p["values"]["E1"] == pytest.approx(2.746)
    assert p["states"].size == 1945


def test_peng_recorded_once_per_file(tmp_path):
    """Two pylons driven by engine 1 print "Peng 1" twice: one value per file (36b7dd7)."""
    pat, rm, d = pattern()
    txt = open(os.path.join(DATA, "point_template")).read()
    txt = txt.replace(" Peng 2 :  2.746", " Peng 1 :  9.999", 1)
    f = tmp_path / "pt"
    f.write_text(txt)
    p = parse_point_file(str(f), pat, rm, wanted=d["ModelVariables"])
    assert p["values"]["E1"] == pytest.approx(2.746)
    d = read_aswing_file(str(f), d, rm, RecordStateHistory=True, compiled_pattern=pat)
    assert d["ModelVariables"]["E1"]["values"] == [pytest.approx(2.746)]


def test_truncated_point_file_is_rejected(tmp_path):
    pat, rm, d = pattern()
    txt = open(os.path.join(DATA, "point_template")).read()
    f = tmp_path / "pt"
    f.write_text(txt[: len(txt) // 2])
    with pytest.raises(IncompletePointFile):
        parse_point_file(str(f), pat, rm)


def test_npz_and_json_roundtrip(tmp_path):
    pat, rm, d = pattern()
    for _ in range(3):
        d = read_aswing_file(os.path.join(DATA, "point_template"), d, rm, RecordStateHistory=True,
                             compiled_pattern=pat)
    d["Metadata"] = {"Dt": 0.01}
    for fmt in ("npz", "json"):
        path = export_data_dict(d, str(tmp_path / "run"), fmt=fmt)
        back = import_data_dict(path)
        assert back["ModelStates"].shape == (3, 1945)
        np.testing.assert_array_equal(back["ModelStates"].astype(np.float32),
                                      np.vstack(d["ModelStates"]).astype(np.float32))
        assert back["ModelVariables"]["Alpha"]["values"] == pytest.approx(d["ModelVariables"]["Alpha"]["values"])
        assert back["ModelVariables"]["Alpha"]["unit"] == d["ModelVariables"]["Alpha"]["unit"]
    assert os.path.getsize(tmp_path / "run.npz") < os.path.getsize(tmp_path / "run.json.gz")


def test_float32_states_keep_aswing_values():
    """ASWING (single precision) prints states with 9 significant digits, which identify one
    float32 value: storing float32 keeps exactly ASWING's values (printing them back gives the
    file's numbers)."""
    pat, rm, d = pattern()
    p = parse_point_file(os.path.join(DATA, "point_template"), pat, rm)
    s32 = p["states"].astype(np.float32)
    back = np.array([float(f"{v:.9g}") for v in s32])
    np.testing.assert_array_equal(back, p["states"])


def test_control_files(tmp_path):
    python2text(str(tmp_path / "c1"), {"F1": 0.5, "E1": 2.0})
    assert (tmp_path / "c1").read_text().splitlines()[0] == "time F1 E1"
    python2text_schedule(str(tmp_path / "c2"), [0.01, 0.02], {"F2": [1.0, 2.0]})
    rows = (tmp_path / "c2").read_text().split("\n")
    assert rows[0] == "time F2" and rows[2] == "0.02\t2.0"


def test_import_without_matlab_or_matplotlib():
    code = ("import sys, WingLoop_Library; "
            "print(any(m in sys.modules for m in ('matlab', 'fmpy', 'matplotlib')))")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         cwd=os.path.dirname(HERE))
    assert out.stdout.strip() == "False", out.stderr


# ----------------------------------------------------------------------------- time march
def test_time_grid_and_no_lost_samples(case, monkeypatch):
    ok, WL, V, S = run(case, N=10, monkeypatch=monkeypatch)
    assert ok and WL.sim_status == "completed"
    assert V["Time"]["values"] == pytest.approx([0.01 * k for k in range(1, 11)])
    assert list(S[:, 0]) == list(range(1, 11))          # every level exactly once, in order


def test_record_initial_and_chunks(case, monkeypatch):
    ok, WL, V, S = run(case, N=10, K=3, record_initial=True, monkeypatch=monkeypatch)
    assert ok
    assert V["Time"]["values"] == pytest.approx([0.0, 0.03, 0.06, 0.09, 0.10])
    assert list(S[:, 0]) == [0, 3, 6, 9, 10]


def test_nonconverged_stops_by_default(case, monkeypatch):
    ok, WL, V, S = run(case, N=10, env={"FAKE_NC": "4"}, monkeypatch=monkeypatch)
    assert not ok and "not converged at step 4" in WL.sim_status
    assert V["IsConverged"]["values"] == [True, True, True, False]
    assert V["NewtonRestarts"]["values"][-1] == 1


def test_retry_bookkeeping(case, monkeypatch):
    ok, WL, V, S = run(case, N=8, retry_substeps=10, env={"FAKE_NC": "4"}, monkeypatch=monkeypatch)
    assert ok
    assert V["Retried"]["values"] == [0, 0, 0, 1, 0, 0, 0, 0]
    # the retry re-marches level 4 in 10 sub-steps (fake: levels 4..13); one record per step, no duplicate
    assert len(S) == 8 and len(set(S[:, 0])) == 8
    assert V["Time"]["values"] == pytest.approx([0.01 * k for k in range(1, 9)])
    assert S[3, 1] == pytest.approx(0.04)                # the retried point is at the same time


def test_retry_that_fails_is_reported(case, monkeypatch):
    ok, WL, V, S = run(case, N=8, retry_substeps=4, env={"FAKE_NC_HARD": "0.03"}, monkeypatch=monkeypatch)
    assert not ok
    assert V["Retried"]["values"] == [0, 0, 2]


def test_hang_is_reported(case, monkeypatch):
    ok, WL, V, S = run(case, N=10, write_timeout=2, env={"FAKE_HANG_LEVEL": "5"}, monkeypatch=monkeypatch)
    assert not ok and WL.sim_status.startswith("hung at step 4")
    assert len(V["Time"]["values"]) == 4


def test_crash_is_reported(case, monkeypatch):
    ok, WL, V, S = run(case, N=10, env={"FAKE_CRASH_LEVEL": "6"}, monkeypatch=monkeypatch)
    assert not ok and WL.sim_status.startswith("crashed at step 5")


def test_controller_timing_and_command_log(case, monkeypatch, tmp_path):
    ctrl = write_controller(tmp_path / "ctrl_a.py",
                            "    def step(self, state, Dt, channels):\n"
                            "        self.calls.append(channels['Time'])\n"
                            "        return {'F2': 100 + 10 * channels['Time'] / 0.01, 'recovery': 1.0}\n")
    ok, WL, V, S = run(case, N=5, controller=ctrl, monkeypatch=monkeypatch)
    assert ok
    # u over [t_k, t_k+Dt] comes from the state at t_k, from t_0 = 0
    assert V["cmd_F2"]["values"] == pytest.approx([100, 110, 120, 130, 140])
    assert V["F2"]["values"] == pytest.approx([100, 110, 120, 130, 140])   # what ASWING applied
    assert V["cmd_recovery"]["values"] == [1.0] * 5
    assert "recovery" not in open(os.path.join(case, "input")).read() if os.path.exists(
        os.path.join(case, "input")) else True


def test_stop_condition(case, monkeypatch):
    ok, WL, V, S = run(case, N=10, stop_condition=lambda ch: ch["Time"] >= 0.03 - 1e-9, monkeypatch=monkeypatch)
    assert not ok and "stop_condition at step 3" in WL.sim_status


def test_open_loop_schedule_and_takeover(case, monkeypatch, tmp_path):
    ctrl = write_controller(tmp_path / "ctrl_b.py",
                            "    def schedule(self, times):\n"
                            "        return {'F2': -1000 * np.asarray(times)}\n"
                            "    def takeover(self, channels):\n"
                            "        return channels['Time'] >= 0.06 - 1e-9\n"
                            "    def step(self, state, Dt):\n"
                            "        return {'F2': 7.0}\n")
    ok, WL, V, S = run(case, N=8, controller=ctrl, monkeypatch=monkeypatch)
    assert ok
    assert V["F2"]["values"] == pytest.approx([-10, -20, -30, -40, -50, -60, 7, 7])
    assert WL.metadata["takeover_time"] == pytest.approx(0.06)


def test_aero_flags(case, monkeypatch):
    ok, WL, V, S = run(case, N=2, aero_flags={"freeze_vl": True, "lag": False}, monkeypatch=monkeypatch)
    assert ok and WL.metadata["aero_flags"] == {"freeze_vl": True, "lag": False}


def test_constraint_table(case, monkeypatch):
    """Keys reach ASWING's constraint table; "c" (unsteady) does not leave the march steady."""
    ok, WL, V, S = run(case, N=3, constraint_keys=["f", "c", "l"], monkeypatch=monkeypatch)
    assert ok and WL.sim_status == "completed"
    assert open(case / "table_keys.log").read().split() == ["f", "c", "l"]
    assert WL.metadata["constraint_keys"] == ["f", "c", "l"]


def test_metadata_and_output(case, monkeypatch):
    ok, WL, V, S = run(case, N=3, monkeypatch=monkeypatch)
    WL.WingLoop_LogFile["ModelStates"] = S
    path = export_data_dict(dict(WL.WingLoop_LogFile, Metadata=WL.metadata), str(case / "out"))
    m = import_data_dict(path)["Metadata"]
    assert m["N"] == 3 and m["Dt"] == 0.01 and m["status"] == "completed"
    assert m["aswing_executable"] == FAKE and len(m["aswing_sha256"]) == 64


def test_filename_guard(case):
    WL = WingLoop()
    with pytest.raises(ValueError, match="80"):
        WL.Launch_ASWING(aswing_fullpath=FAKE, sim_directory=str(case), asw_filename="case.asw",
                         output_filename="o" * 81)


def test_too_many_steps(case, monkeypatch):
    WL = WingLoop()
    WL.Launch_WingLoop_Control(None, None, timestep=0.01)
    WL.Launch_ASWING(aswing_fullpath=FAKE, sim_directory=str(case), asw_filename="case.asw")
    with pytest.raises(ValueError, match="NPNTX"):
        WL.Time_Transient_Simulation(0.01, 20000, 1)
    WL.Closing_WingLoop()
