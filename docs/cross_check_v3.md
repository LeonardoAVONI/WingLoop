# WingLoop 3.0 backend cross-check

Test aircraft, 1-cosine gust, N = 300, Dt = 0.01 s, K = 1, ASWING `/home/leonardo-avoni/Desktop/02_ASWING/ASWING_Stable/bin/aswing`.
Differences against the Python run. Timings: one run each, this machine.

| backend | verdict | max dF2 [deg] | max dstate / range | shifted by 1 step | ms/step | vs fixed stick | overhead [s] | status |
|---|---|---|---|---|---|---|---|---|
| fixed | reference only | 3.46e+00 | 8.27e+02 | 8.27e+02 | 52.3 | +0.0 % | 0.05 | completed |
| py | reference | - | - | - | 52.3 | +0.0 % | 0.08 | completed |
| mat | PASS | 0.00e+00 | 0.00e+00 | 1.00e+00 | 55.1 | +5.4 % | 4.75 | completed |
| sim | PASS | 0.00e+00 | 0.00e+00 | 1.00e+00 | 514.2 | +883.5 % | 29.36 | completed |
| fmu_rebuild | PASS | 0.00e+00 | 0.00e+00 | 1.00e+00 | 54.7 | +4.6 % | 52.38 | completed |
| fmu | PASS | 0.00e+00 | 0.00e+00 | 1.00e+00 | 52.6 | +0.6 % | 0.13 | completed |
| simv2 | PASS | 0.00e+00 | 0.00e+00 | 1.00e+00 | 58.3 | +11.5 % | 40.32 | completed |
| native | reference only | 3.46e+00 | 8.27e+02 | 8.27e+02 | 48.5 | -7.2 % | 0.06 | completed |
