# Timing benchmark on another machine (thesis "Time Performance" section)

`benchmark.py` times every WingLoop configuration on the test aircraft (Murua HALE, 30 m/s,
20 km density, 1-cosine gust, H = 15 m, 3 m/s, 300 steps of 10 ms) with the thesis PID gains
(`--scenario v1_k1`: Kp 7, Ki 30 1/s, Kd 8 s in deg/deg; `py_k10` uses the K = 10 gains 2, 0.5, 0.5).

## Before running

1. `git pull` this repository on branch `v3`; `pip install -e .` in the Python environment that
   has the MATLAB Engine for Python (the one WingLoop's MATLAB/Simulink backends use).
2. ASWING_Stable built and reachable through the alias `aswing_stable` (`~/.bashrc`), or pass
   `--aswing /path/to/aswing`.
3. MATLAB with Simulink and Simulink Coder (FMU export).
4. Laptop plugged in, power profile `performance` (`powerprofilesctl set performance`), other
   programs closed. The script checks these and stops on a warning (`--force` overrides).

## Run

```bash
cd WingLoop_Library/wingloop_testrun
python benchmark.py --quick --force     # 2-3 min smoke test (1 repeat, 50 steps): check it ends cleanly
python benchmark.py                     # the benchmark: 5 repeats, plot off and on, about 1-1.5 h
```

The script first re-trims the aircraft with this machine's ASWING and stops if the trim differs from
the reference (theta 4.6919 deg, F2 -4.639 deg, E1 2.746 N): a different ASWING build would change
the test case. Live-plot windows open during the plot-on runs; do not use the machine meanwhile.

## Check the results

At the end the script writes `validation_check.pdf` (same layout as the thesis validation figure,
grey band = wing inside the gust) with the reference runs made on the Dell 7550 dotted on top, and
`validation_check.md` with the largest difference per run and channel. The curves should overlap:
differences of about 1e-4 or less are round-off between machines; larger ones mean a different
ASWING build, files or settings, so stop there before using the timings.

The same runs also give the thesis figures `Murua_validation.pdf` and `Murua_validation_error.pdf`
(vector, thesis style, fixed stick / ASWING control / WingLoop K = 1 / K = 10 with the grey gust band),
written by `make_murua_validation.py` into the output folder.

## Bring back

Copy the whole output folder `results_bench/<host>_<date>/` (environment.json, runs.csv,
summary.md, summary.json, timing_bars.pdf, runs/ with the result files of the first repeat).
