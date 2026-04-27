# =============================================================================
# WingLoop — PyControl_IO
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
WingLoop Library — PyControl_IO
Input / Output Utilities for ASWING-WingLoop Data Exchange
====================================================================================

Author: Leonardo Avoni
Email: avonileonardo@gmail.com
Initial release: 25 Oct 2024
Last modified: 10 Mar 2026

------------------------------------------------------------------------------------
Overview
------------------------------------------------------------------------------------
PyControl_IO provides input/output utilities used by the WingLoop framework to
interface PyControl and other libraries with ASWING program.

The module is responsible for:

    • Parsing ASWING output files
    • Building and maintaining simulation data dictionaries
    • Exporting simulation results to structured formats (JSON)
    • Importing previously saved simulation datasets
    • Writing control command files compatible with ASWING
    • Providing debugging and diagnostic summaries

These utilities allow WingLoop to convert ASWING text-based outputs into
structured Python objects suitable for analysis, control algorithms, and
visualization.

------------------------------------------------------------------------------------
Core Data Structure
------------------------------------------------------------------------------------
Most functions operate on a **simulation data dictionary** with the structure:

    data_dict = {
        "ModelName": str,
        "ModelStates": list | numpy.ndarray,
        "ModelVariables": {
            variable_name: {
                "values": list,
                "unit": str | None,
                "latex": str | None
            }
        }
    }

This structure stores all time-series variables extracted from ASWING output
files, including aircraft states, aerodynamic quantities, and control signals.

------------------------------------------------------------------------------------
Main Functionalities
------------------------------------------------------------------------------------

initialize_data_dict(...)
    Creates the base simulation dictionary used to store parsed variables.

read_aswing_file(...)
    Parses an ASWING output file and appends the extracted values to the
    provided data dictionary.

python2text(...)
    Generates ASWING-compatible time-history control files from Python
    control dictionaries.

export_data_dict(...)
    Serializes a simulation dictionary to a human-readable JSON file.

import_data_dict(...)
    Restores a previously exported JSON dataset and reconstructs the
    dictionary structure used by WingLoop.

print_aswing_summary(...)
    Displays a formatted summary of parsed variables and state history.

------------------------------------------------------------------------------------
Utility Functions
------------------------------------------------------------------------------------

seconds2hms(...)
    Converts a time expressed in seconds into hours, minutes, and seconds.

scientific_to_decimal(...)
    Converts scientific notation strings into decimal representation.
    Useful when exporting values to environments that do not handle
    scientific notation consistently (e.g., MATLAB parsing issues).

------------------------------------------------------------------------------------
ASWING Parsing Notes
------------------------------------------------------------------------------------

• ASWING output files are text-based and not strictly structured, which
  requires robust parsing strategies.

• Variable names may contain spaces (e.g., "earth X", "Dyn.Pr."), so
  parsing uses dynamic regular expressions constructed from requested
  variable names.

• The "Op.Point" variable is not iterated beyond 999 because ASWING
  prints values above this threshold as "***".

• Incorrectly formatted WingLoop input files may not be recognized
  correctly by ASWING.

------------------------------------------------------------------------------------
Testing
------------------------------------------------------------------------------------
The module includes a standalone test block demonstrating:

    • dictionary initialization
    • ASWING file parsing
    • control-file generation
    • JSON export / import of simulation datasets

Run directly:

    python PyControl_IO.py

------------------------------------------------------------------------------------
Role within WingLoop
------------------------------------------------------------------------------------
PyControl_IO acts as the **data interface layer** between ASWING simulations and
the WingLoop Python ecosystem.

Typical workflow:

    ASWING output file test_file
        ↓
    read_aswing_file(test_file)
        ↓
    structured data dictionary is updated
        ↓
    WingLoop operation happen
        visualization (PyControl_LivePlotter)
        control algorithms (PyControl)
        PyControl defines the new control signals
    New control signals are available
        ↓
    python2text(new_control_signals)
        this writes the input file
        ↓
    the input file is fed back to ASWING
    
    At the end of the simulation:   
        analysis / export

====================================================================================
"""

import numpy as np
import re
import pprint

def initialize_data_dict(requested, rename_map=None, latex=None, N_steps = None):
    if rename_map is None:
        rename_map = {}
    if latex is None:
        latex = {}
    units = {}
    
    data = {
        "ModelName": None,
        "ModelStates": [],        # becomes a 2D numpy array after first state is seen
        "_N_steps":    N_steps,   # None = unknown, fall back to list append
        "_state_count": 0,        # row index for preallocated array
        "ModelVariables": {}
    }

    # Use the final (clean) internal name as dict key for consistency with units/latex
    for raw_var in requested:
        internal_var = rename_map.get(raw_var, raw_var)
        data["ModelVariables"][internal_var] = {
            "values": [],#np.empty(num)
            "unit": units.get(internal_var) or units.get(raw_var),
            "latex": latex.get(internal_var) or latex.get(raw_var)
        }

    return data

#def initialize_control_dict(control_elements, default_value=0.0):
#    """
#    Creates a control dictionary with all requested control elements set to a default value.
#    
#    Example:
#        control = initialize_control_dict(["F1", "F2", "F3", "F4", "E1", "E2"])
#        # → {'F1': 0.0, 'F2': 0.0, 'F3': 0.0, 'F4': 0.0, 'E1': 0.0, 'E2': 0.0}
#    """
#    return {elem: default_value for elem in control_elements}


def _build_pattern(data_dict, rename_map):
    if rename_map is None:
        rename_map = {}
    raw_names = list(rename_map.keys())
    for internal in list(data_dict["ModelVariables"].keys()):
        if internal not in rename_map.values():
            raw_names.append(internal)
    all_raw = set(raw_names)
    sorted_raw = sorted(all_raw, key=len, reverse=True)
    name_alt = '|'.join(re.escape(name) for name in sorted_raw)
    first_words = {name.split()[0] for name in all_raw if name}
    sorted_first = sorted(first_words, key=len, reverse=True)
    first_alt = '|'.join(re.escape(w) for w in sorted_first)
    return re.compile(rf"""
        (?<![A-Za-z0-9])
        ({name_alt})
        \s*:\s*
        ([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)
        (?:\s+
            (?! (?:{first_alt})(?=[\s:]|$))
            ([^\s:]+)
            (?= \s (?! :) | $ )
        )?
    """, re.VERBOSE)

def read_aswing_file(filepath, data_dict, rename_map=None,
                     RecordStateHistory=False, compiled_pattern=None):

    if rename_map is None:
        rename_map = {}
    pattern = compiled_pattern if compiled_pattern is not None else _build_pattern(data_dict, rename_map)

    with open(filepath, "r") as f:
        lines = f.readlines()

    data_dict["ModelName"] = lines[2].strip()

    recording_states = False
    states = []
    converged = False

    for line in lines:
        stripped = line.strip()

        if "Status:" in stripped:
            converged = "Converged" in stripped

        if "STARTEDPRINTINGSTATES" in stripped:
            recording_states = True
            states = []
            continue
        #if "ENDEDPRINTINGSTATES" in stripped:
        #    recording_states = False
        #    if RecordStateHistory:
        #        data_dict["ModelStates"].append(np.array(states))
        #    else:
        #        data_dict["ModelStates"] = np.array(states)
        #    continue
        if "ENDEDPRINTINGSTATES" in stripped:
            recording_states = False
            state_array = np.array(states, dtype=np.float64)
            if RecordStateHistory:
                N   = data_dict.get("_N_steps")
                idx = data_dict.get("_state_count", 0)

                if N is not None:
                    # First call: preallocate now that we know state length
                    if idx == 0:
                        data_dict["ModelStates"] = np.empty((N, len(state_array)), dtype=np.float64)
                    data_dict["ModelStates"][idx] = state_array
                    data_dict["_state_count"] = idx + 1
                else:
                    # N unknown: original list-of-arrays fallback
                    if not isinstance(data_dict["ModelStates"], list):
                        data_dict["ModelStates"] = list(data_dict["ModelStates"])
                    data_dict["ModelStates"].append(state_array)
            else:
                data_dict["ModelStates"] = state_array
            continue
        if recording_states:
            try:
                states.extend(map(float, stripped.split()))
            except:
                pass
            continue

        # Extract variables
        for match in pattern.finditer(line):
            var_name = match.group(1).strip()
            value = float(match.group(2))
            unit_candidate = match.group(3)

            internal_name = rename_map.get(var_name, var_name)

            if internal_name in data_dict["ModelVariables"]:
                data_dict["ModelVariables"][internal_name]["values"].append(value)

                if (data_dict["ModelVariables"][internal_name]["unit"] is None and
                        unit_candidate is not None):
                    data_dict["ModelVariables"][internal_name]["unit"] = unit_candidate

    if "IsConverged" in data_dict["ModelVariables"]:
        data_dict["ModelVariables"]["IsConverged"]["values"].append(converged)

    return data_dict

def print_aswing_summary(data_dict, max_vars_per_line=4):
    print("\n" + "="*70)
    print("ASWING PARSER SUMMARY")
    print("="*70)

    print(f"\nModel Name      : {data_dict.get('ModelName')}")

    model_states = data_dict.get("ModelStates")
    if isinstance(model_states, list):
        print(f"ModelStates history length: {len(model_states)}")
        if model_states:
            last_states = model_states[-1]
            print(f"Last ModelStates length: {len(last_states)}")
            print(f"Last ModelStates preview: {last_states[:5]} ... {last_states[-5:]}")
        else:
            print("ModelStates          : Empty history")
    elif model_states is not None:
        print(f"ModelStates length   : {len(model_states)}")
        print(f"ModelStates preview  : {model_states[:5]} ... {model_states[-5:]}")
    else:
        print("ModelStates          : None")

    print("\nModel Variables:")
    print("-"*70)

    counter = 0
    for name, content in data_dict["ModelVariables"].items():
        values = content["values"]
        unit = content["unit"]

        nvals = len(values)
        last_val = values[-1] if nvals > 0 else None

        if isinstance(last_val, (int, float)):
            last_str = f"{last_val:.6g}"
        else:
            last_str = str(last_val)

        unit_str = unit if unit is not None else "-"

        print(f"{name:<15} | n={nvals:<4} | last={last_str:<15} | unit={unit_str}")

        counter += 1
        if counter % max_vars_per_line == 0:
            print()

    if "IsConverged" in data_dict["ModelVariables"]:
        conv = data_dict["ModelVariables"]["IsConverged"]["values"]
        print("\nConvergence history:")
        print(conv)

    print("\n" + "="*70)



def seconds2hms(seconds):
    """
    Converts a time expressed in seconds -typically obtained using time.time()- to a time in h, m, s
    """
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    remaining_seconds = np.round(seconds % 60)
    return hours, minutes, remaining_seconds

def scientific_to_decimal(sci_str):
    """ 
    Convert the scientific notation string to a regular float and format it as a string
    This function is used by "convert_extracted_values"
    We had to convert from 1E5 to 100000 because otherwise the numbers could not be used by Matlab
    """
    num = float(sci_str)
    return f"{num:.10f}".rstrip("0").rstrip(".")



def python2text(filename, control):
    """
    Writes an ASWING time-history control file using the provided control dictionary.
    
    The header will be exactly: "time F1 F2 ..." (no leading space after "time").
    
    Parameters:
        filename : str, path to the output control file
        control  : dict with control keys ("F1", "F2", "E1", etc.) and their values
                   The order of columns = order of keys in the dictionary
    """
    # Get the control names in the order they appear in the dict
    control_names = list(control.keys())
    
    # Header: "time F1 F2 F3 ..." (no space before first control name)
    header = "time " + " ".join(control_names) + "\n"
    
    # Convert values to strings (ASWING expects plain numbers)
    values = [str(value) for value in control.values()]
    
    # Two constant time steps (common simple format)
    line1 = "0.0\t" + "\t".join(values) + "\n"
    line2 = "1000.0\t" + "\t".join(values)
    
    # Combine
    content = header + line1 + line2
    
    with open(filename, "w") as f:
        f.write(content)

def export_data_dict(data_dict, filepath, compress=True):
    """
    Serialise *data_dict* to a JSON file, optionally gzip-compressed.

    numpy arrays (ModelStates entries) are converted to nested Python lists so
    that the file is plain JSON with no binary blobs.  All other values are
    already JSON-compatible (str, float, bool, list, None).

    Parameters
    ----------
    data_dict : dict   – the dictionary to export
    filepath  : str    – destination path, e.g. "results/run01"
                         Extension is added automatically:
                           compress=True  → .json.gz
                           compress=False → .json
    compress  : bool   – gzip the output (default: True)

    Returns
    -------
    filepath : str     – the path actually written
    """
    print("[IO FILEPATH]",filepath)
    import json, gzip, os

    # Strip any existing extension, then re-apply the correct one
    for suffix in (".json.gz", ".json"):
        if filepath.endswith(suffix):
            filepath = filepath[: -len(suffix)]
            break
    filepath += ".json.gz" if compress else ".json"

    os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)

    def _make_serialisable(obj):
        """Recursively convert numpy types / arrays to plain Python."""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, (np.bool_,)):
            return bool(obj)
        if isinstance(obj, list):
            return [_make_serialisable(v) for v in obj]
        if isinstance(obj, dict):
            return {k: _make_serialisable(v) for k, v in obj.items()}
        return obj  # str, float, int, bool, None → already fine

    # Filter pre-allocation keys, then serialise
    serialisable = {k: v for k, v in data_dict.items() if not k.startswith("_")}
    serialisable = _make_serialisable(serialisable)

    json_bytes = json.dumps(serialisable, indent=2, ensure_ascii=False).encode("utf-8")

    if compress:
        with gzip.open(filepath, "wb", compresslevel=6) as f:
            f.write(json_bytes)
    else:
        with open(filepath, "wb") as f:
            f.write(json_bytes)

    n_vars   = len(data_dict.get("ModelVariables", {}))
    n_states = len(data_dict.get("ModelStates", []))
    size_kb  = os.path.getsize(filepath) / 1024
    print(f"[PyControl_IO] Saved → {filepath}  "
          f"({n_vars} variables, {n_states} state snapshot(s), {size_kb:.1f} KB)")
    return filepath

def import_data_dict(filepath):
    import json, gzip, os

    # Strip any known extension to get the bare stem
    stem = filepath
    for suffix in (".json.gz", ".json"):
        if stem.endswith(suffix):
            stem = stem[: -len(suffix)]
            break

    # Try compressed first, then plain
    candidates = [stem + ".json.gz", stem + ".json"]

    resolved = next((p for p in candidates if os.path.exists(p)), None)
    if resolved is None:
        raise FileNotFoundError(
            f"[PyControl_IO] Could not find file for: {filepath}\n"
            f"  Tried: {candidates}"
        )

    if resolved.endswith(".gz"):
        with gzip.open(resolved, "rb") as f:
            data_dict = json.loads(f.read().decode("utf-8"))
    else:
        with open(resolved, "r", encoding="utf-8") as f:
            data_dict = json.load(f)

    # Restore numpy arrays for ModelStates
    raw_states = data_dict.get("ModelStates")
    if isinstance(raw_states, list) and raw_states:
        if isinstance(raw_states[0], list):
            data_dict["ModelStates"] = [np.array(s) for s in raw_states]
        else:
            data_dict["ModelStates"] = np.array(raw_states)

    n_vars   = len(data_dict.get("ModelVariables", {}))
    n_states = len(data_dict.get("ModelStates", []))
    print(f"[PyControl_IO] Loaded ← {resolved}  "
          f"({n_vars} variables, {n_states} state snapshot(s))")
    return data_dict

if __name__=="__main__":
    requested = [

        # Position & Attitude
        "Time","earth X","earth Y","earth Z",
        "Heading","Elev.","Bank",
        "Alpha","Beta","Velocity",

        # Angular velocities / accelerations
        "Wx","Wy","Wz",
        "Wdotx","Wdoty","Wdotz",

        # Linear velocities / accelerations
        "Ux","Uy","Uz",
        "Udotx","Udoty","Udotz",

        # Moments
        "sum Mx","sum My","sum Mz",

        # Forces
        "sum Fx","sum Fy","sum Fz",

        # Aero & reference quantities
        "Lift","Density","Ref.Area",
        "Weight","Dyn.Pr.","Ref.Span",
        "Load Fac","VIAS","Ref.Chrd",
        "Mach","VTAS","MachPG",

        # Aero coefficients
        "CL","CD","L/D","Cl'",
        "Cm","CDi","e","Cn'",

        # Convergence (we will treat specially)
        "IsConverged",
        
        "Op.Point",
        "altitude"
    ]



    rename_map = {
#        "earth X": "earthX",
#        "earth Y": "earthY",
#        "earth Z": "earthZ",
#        "Elev.": "Pitch"
    }

    latex = {
        "Time": r"$t$",
        "earthX": r"$X_E$",
        "Velocity": r"$V$"
    }

    adimensional_vars = {
        "CL", "CD", "Cm", "Cn'",
        "Mach", "MachPG", "Load Fac","e"
    }

    control_elements = ["F1", "F2", "F3", "F4", "E1", "E2"]

    for ctrl in control_elements:
        if ctrl.startswith("F"):
            num = ctrl[1:]
            raw = f"Flap {num}"
        elif ctrl.startswith("E"):
            num = ctrl[1:]
            raw = f"Peng {num}"
        else:
            raise ValueError(f"Unknown control prefix in {ctrl}")
        requested.append(raw)
        rename_map[raw] = ctrl
        adimensional_vars.add(ctrl)

    data_metric = initialize_data_dict(requested, rename_map, latex)   # ← added rename_map here

    for i in range(3):
        data_metric = read_aswing_file("test_files/test_output/ASWING_test_output_metric", data_metric, rename_map,RecordStateHistory=True)
    print_aswing_summary(data_metric)
    


    data_imperial = initialize_data_dict(requested, rename_map, latex)   # ← added rename_map here
    data_imperial = read_aswing_file("test_files/test_output/ASWING_test_output_imperial", data_imperial, rename_map)
    print_aswing_summary(data_imperial)
        
    # Example 1: All controls provided
    control = {
        "F1": 0.00710374909,
        "F2": -15.3360357,
        "F3": 3.97384977,
        "F4": -0.423270404,
        "E1": 9.5,
        "E2": 10.5
    }

    python2text("controls", control)
    # → writes file with columns: time F1 F2 F3 E1 E2
    
    export_data_dict(data_metric,"data_imperial.json")
    a = import_data_dict("data_imperial.json")
    print(a)
    export_data_dict(a,"data_imperial.json",compress=False)
    #pprint.pprint(data_metric)
    """  

    {'ModelName': 'Flexible HALE 3, Murua et al. 2012',
    'ModelStates': [array([ 7.10374909e-03, -1.53360357e+01,  3.97384977e+00, ...,
        -4.23310101e-01,  9.50000000e+00,  1.05000000e+01]),
                    array([ 7.10374909e-03, -1.53360357e+01,  3.97384977e+00, ...,
        -4.23310101e-01,  9.50000000e+00,  1.05000000e+01]),
                    array([ 7.10374909e-03, -1.53360357e+01,  3.97384977e+00, ...,
        -4.23310101e-01,  9.50000000e+00,  1.05000000e+01])],
    'ModelVariables': {'Alpha': {'latex': None,
                                'unit': 'deg',
                                'values': [5.424489498138428,
                                            5.424489498138428,
                                            5.424489498138428]},
                        'Bank': {'latex': None,

    """