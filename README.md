
## WingLoop 2.0
<div align="center">
  <img src="./docs/wingLoop_image.jpg" alt="WingLoop GUI during a simulation run" width="100%" />
</div>


**Author:** Leonardo Avoni  
**Contact:** avonileonardo@gmail.com  
**Last Modified:** 10/03/2026  
**Platform:** Ubuntu  
**Tested with:** ASWING 5.98  

---

## Overview

WingLoop is a closed-loop control framework that extends the capabilities of [ASWING](https://web.mit.edu/drela/Public/web/aswing/), a structural and aerodynamic analysis tool for flexible aircraft. While ASWING natively supports only linear bi-scheduled controllers, WingLoop integrates ASWING with Python — and optionally MATLAB or Simulink — to enable arbitrary control laws, including nonlinear strategies.

The core of WingLoop is a coupling between ASWING and external control environments. The simulation advances in a time-marching loop, either step by step or in chunks of *K* timesteps. Between iterations, the simulation state is passed to Python, and then to MATLAB or Simulink if needed (through the corresponding API), where the control laws are computed. The resulting control inputs are returned to ASWING for the next timestep, enabling closed-loop simulations with arbitrary controllers.

### Reference Publication

> Leonardo Avoni, Murat Bronz, Jean-Philippe Condomines and Jean-Marc Moschetta.  
> *"Enhancing ASWING Flight Dynamics Simulations with Closed-Loop Control for Flexible Aircraft,"*  
> AIAA 2025-3425. AIAA AVIATION FORUM AND ASCEND 2025. July 2025.  
> https://arc.aiaa.org/doi/10.2514/6.2025-3425

---

## Installation

From the `02_WingLoop` folder, run:

```bash
pip install -e .
```

After installation, the following imports will be available:

```python
from WingLoop_Library import Aswing_Director
from WingLoop_Library import WingLoop
```

---

## Quick Start

Navigate to the test run directory and execute the test script:

```bash
cd WingLoop_Library/wingloop_testrun
python wingloop_testrun.py
```
The test run simulates the aircraft shown below, featuring 4 groups of control surfaces and two engines.

<div align="center">
  <img src="./docs/wingloop_test_aircraft.png" alt="Test aircraft configuration" width="100%" />
  <p><em>Test aircraft configuration</em></p>
</div>

<div align="center">
  <img src="./docs/wingloop_test.png" alt="WingLoop GUI during a simulation run" width="100%" />
  <p><em>WingLoop GUI during a simulation run</em></p>
</div>

A sample `.json` output from a completed run is available [here](docs/wingloop_test_aircraft.json).



Individual library scripts (`PyControl`, `PyControl_Plot`,...) can also be run and tested independently via their `__main__` block:

```python
if __name__ == "__main__":
    ...
```



---

## Repository Structure

```
WingLoop_Library/
├── Aswing_Director.py          # Communicates with ASWING (sends commands)
├── WingLoop.py                 # Main loop director — orchestrates the full WingLoop execution
├── PyControl.py                # Handles control logic: receives states and dispatches to control laws
├── PyControl_IO.py             # Reads ASWING state files; writes control input files for ASWING
├── PyControl_Plot.py           # Real-time and post-run plotting utilities
├── PyControl_additional.py     # Legacy code from earlier WingLoop versions
├── __init__.py
│
├── test_files/
│   ├── readme
│   ├── test_aircraft/
│   ├── test_controllers/
│   │   ├── matlab/
│   │   ├── python/
│   │   ├── simulink/
│   └── test_output/
│
└── wingloop_testrun/
    ├── aswing_geometry/
    ├── matlab_controller/
    ├── python_controller/
    ├── simulink_controller/
    └── wingloop_testrun.py
```

---

## Library Components

| Module | Description |
|---|---|
| `WingLoop.py` | Top-level director of the WingLoop execution loop |
| `Aswing_Director.py` | Interface to ASWING; sends commands to the simulator |
| `PyControl.py` | Receives aircraft states from ASWING and dispatches to control laws (Python, MATLAB, Simulink) |
| `PyControl_IO.py` | I/O utilities: reads ASWING state files, writes control input files |
| `PyControl_Plot.py` | Plotting utilities for real-time monitoring and post-run analysis |
| `PyControl_additional.py` | Legacy utilities from earlier versions |

Further details on each module are available in the header of the respective source file.

---

## Supported Controller Types

- **Python** — native Python control laws
- **MATLAB** — via MATLAB's Python API
- **Simulink** — via FMU export or MATLAB/Simulink API

---

## License

WingLoop is licensed under **[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)** — free for non-commercial use with attribution.

**Commercial use is not permitted under this license.** If you wish to use WingLoop in a commercial context, please contact the author to discuss a separate agreement:
**Leonardo Avoni** — avonileonardo@gmail.com

If you use WingLoop in academic work, please cite:
> Leonardo Avoni, Murat Bronz, Jean-Philippe Condomines and Jean-Marc Moschetta.
> *"Enhancing ASWING Flight Dynamics Simulations with Closed-Loop Control for Flexible Aircraft,"*
> AIAA 2025-3425. AIAA AVIATION FORUM AND ASCEND 2025. July 2025.
> https://arc.aiaa.org/doi/10.2514/6.2025-3425

See [`LICENSE`](LICENSE) and [`CONTRIBUTING.md`](CONTRIBUTING.md) for full details.
