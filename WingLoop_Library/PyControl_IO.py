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
        "ModelParameters": {},
        "ModelStates": [],        # becomes a 2D numpy array after first state is seen
        "_N_steps":    N_steps,   # None = unknown, fall back to list append
        "_state_count": 0,        # row index for preallocated array
        "ModelVariables": {}
    }

    data["ModelParameters"]["IITOT"] = None
    data["ModelParameters"]["NJOIN"] = None
    data["ModelParameters"]["NNTOT"] = None
    data["ModelParameters"]["KPFREE"] = None
    data["ModelParameters"]["KPTOT"] = None

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




















class IncompletePointFile(RuntimeError):
    """A point file does not end with ENDEDPRINTINGSTATES (truncated or still being written)."""


def parse_point_file(filepath, compiled_pattern, rename_map=None, wanted=None, require_complete=True):
    """
    Parse one ASWING point file (OPER "W") without modifying any data dictionary.

    Parameters
    ----------
    filepath : str
    compiled_pattern : re.Pattern
        From _build_pattern(data_dict, rename_map).
    rename_map : dict | None
        Raw ASWING name -> internal name.
    wanted : container | None
        Internal names to keep (None = all matches).
    require_complete : bool
        Raise IncompletePointFile if the file does not contain ENDEDPRINTINGSTATES
        (a truncated read caused false "not converged" steps under parallel load).

    Returns
    -------
    dict with keys
        "name"      : model name (3rd line)
        "converged" : bool ("Status: Converged")
        "values"    : {internal_name: float}, first match per variable (a variable printed
                      once per engine pylon, "Peng k", is kept once)
        "units"     : {internal_name: unit or None}
        "states"    : 1-D float64 array of the STARTED/ENDEDPRINTINGSTATES block (or None)
        "params"    : {"IITOT": .., "NJOIN": .., ...} from "STATEFILE SIZE:"
    """
    if rename_map is None:
        rename_map = {}
    with open(filepath, "r") as f:
        text = f.read()
    if require_complete and "ENDEDPRINTINGSTATES" not in text[-512:]:
        raise IncompletePointFile(f"{filepath}: no ENDEDPRINTINGSTATES at the end of the file")
    head, found, rest = text.partition("STARTEDPRINTINGSTATES")
    lines = head.split("\n", 4)
    out = {"name": lines[2].strip() if len(lines) > 2 else None, "converged": False,
           "values": {}, "units": {}, "states": None, "params": {}}
    m = _STATUS_RE.search(head)
    if m:
        out["converged"] = "Converged" in m.group(1)
    size_at = head.find("STATEFILE SIZE:")
    if size_at >= 0:
        for k, v in _SIZE_RE.findall(head[size_at + 15:]):
            out["params"][k] = int(float(v))
        head = head[:size_at]
    tables = _TABLE_RE.search(head)          # spanwise beam tables: no global variables there
    if tables:
        head = head[:tables.start()]
    vals, units = out["values"], out["units"]
    for match in compiled_pattern.finditer(head):
        raw = match.group(1).strip()
        internal = rename_map.get(raw, raw)
        if internal in vals or (wanted is not None and internal not in wanted):
            continue
        vals[internal] = float(match.group(2))
        units[internal] = match.group(3)
    if found:
        block = rest.partition("ENDEDPRINTINGSTATES")[0]
        # numeric lines only (section headers such as "Joints:" are skipped, as in V2)
        num = [ln for ln in block.split("\n") if ln.strip() and _NUMLINE_RE.match(ln)]
        try:
            out["states"] = np.array(" ".join(num).split(), dtype=np.float64)
        except ValueError:                  # a mixed line: fall back to the V2 rule, line by line
            states = []
            for ln in num:
                try:
                    states.extend(float(x) for x in ln.split())
                except ValueError:
                    pass
            out["states"] = np.array(states, dtype=np.float64)
    return out


_STATUS_RE = re.compile(r"Status:\s*([^\n]*)")
_SIZE_RE = re.compile(r"^\s*([A-Za-z]\w*)\s*:\s*([-+0-9.Ee]+)\s*$", re.M)
_NUMLINE_RE = re.compile(r"^\s*[-+.0-9]")
_TABLE_RE = re.compile(r"^\s*(?:Surface|Fuselage)\s+Beam\s+\d+:", re.M)


def read_aswing_file(filepath, data_dict, rename_map=None,
                     RecordStateHistory=False, compiled_pattern=None):
    """
    Parse an ASWING point file and append its values to `data_dict` (V2 interface).

    One value per requested variable and file (first match: the Peng fix), "IsConverged"
    from the Status line, the state block appended to ModelStates (RecordStateHistory=True,
    preallocated if data_dict["_N_steps"] is set) or stored as the last state.
    """
    if rename_map is None:
        rename_map = {}
    pattern = compiled_pattern if compiled_pattern is not None else _build_pattern(data_dict, rename_map)
    data_dict.setdefault("ModelParameters", {})
    p = parse_point_file(filepath, pattern, rename_map, wanted=data_dict["ModelVariables"],
                         require_complete=False)
    data_dict["ModelName"] = p["name"]
    data_dict["ModelParameters"].update(p["params"])
    for name, value in p["values"].items():
        var = data_dict["ModelVariables"][name]
        var["values"].append(value)
        if var["unit"] is None and p["units"].get(name) is not None:
            var["unit"] = p["units"][name]
    if p["states"] is not None:
        _store_state(data_dict, p["states"], RecordStateHistory)
    if "IsConverged" in data_dict["ModelVariables"]:
        data_dict["ModelVariables"]["IsConverged"]["values"].append(p["converged"])
    return data_dict


def _store_state(data_dict, state_array, record_history=True):
    """Append one state vector to data_dict["ModelStates"] (preallocated when _N_steps is set)."""
    if not record_history:
        data_dict["ModelStates"] = state_array
        return
    N = data_dict.get("_N_steps")
    idx = data_dict.get("_state_count", 0)
    if N is not None and (idx == 0 or (isinstance(data_dict.get("ModelStates"), np.ndarray)
                                       and data_dict["ModelStates"].ndim == 2)):
        if idx == 0:
            data_dict["ModelStates"] = np.empty((N, len(state_array)), dtype=np.float64)
        if idx >= data_dict["ModelStates"].shape[0]:            # grow if N was too small
            extra = np.empty_like(data_dict["ModelStates"])
            data_dict["ModelStates"] = np.vstack([data_dict["ModelStates"], extra])
        data_dict["ModelStates"][idx] = state_array
    else:
        if not isinstance(data_dict.get("ModelStates"), list):
            data_dict["ModelStates"] = []
        data_dict["ModelStates"].append(state_array)
    data_dict["_state_count"] = idx + 1


def model_states_array(data_dict):
    """ModelStates as one 2-D array (rows = recorded steps), trimmed to the recorded rows."""
    S = data_dict.get("ModelStates")
    if isinstance(S, np.ndarray) and S.ndim == 2:
        return S[: data_dict.get("_state_count", S.shape[0])]
    if isinstance(S, list) and S:
        return np.vstack([np.asarray(s, dtype=np.float64) for s in S])
    if isinstance(S, np.ndarray) and S.ndim == 1 and S.size:
        return S[None, :]
    return np.empty((0, 0))


def print_aswing_summary(data_dict, max_vars_per_line=4):
    print("\n" + "="*70)
    print("ASWING PARSER SUMMARY")
    print("="*70)

    print(f"\nModel Name      : {data_dict.get('ModelName')}")
    print("Model Parameters:")
    print("     IITOT",data_dict["ModelParameters"]["IITOT"]   )
    print("     NJOIN",data_dict["ModelParameters"]["NJOIN"]   )
    print("     NNTOT",data_dict["ModelParameters"]["NNTOT"]   )
    print("     KPFREE", data_dict["ModelParameters"]["KPFREE"])
    print("     KPTOT", data_dict["ModelParameters"]["KPTOT"]  )
    print("\n")
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

def python2text_schedule(filename, times, controls):
    """
    Write an ASWING time-history control file with one row per time.

    ASWING evaluates the trace at the absolute time of each new time level (spline
    through the rows), so a row (t_{k+1}, u_k) per step applies u_k exactly over the
    step that ends at t_{k+1}.

    times    : sequence of absolute times [s]
    controls : {name: sequence of values, same length as times}
    """
    names = list(controls.keys())
    cols = [np.asarray(controls[n], dtype=float) for n in names]
    if any(len(c) != len(times) for c in cols):
        raise ValueError("every control column needs one value per time")
    if len(times) > 9999:
        raise ValueError("ASWING reads at most 10000 rows (NSPX) per control file")
    rows = ["time " + " ".join(names)]
    for i, t in enumerate(times):
        rows.append(repr(float(t)) + "\t" + "\t".join(repr(float(c[i])) for c in cols))
    with open(filename, "w") as f:
        f.write("\n".join(rows) + "\n")


def _serialisable(obj):
    """Recursively convert numpy types / arrays to plain Python (for JSON)."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, (list, tuple)):
        return [_serialisable(v) for v in obj]
    if isinstance(obj, dict):
        return {k: _serialisable(v) for k, v in obj.items()}
    return obj


def export_data_dict(data_dict, filepath, compress=True, fmt=None):
    """
    Save a WingLoop data dictionary.

    fmt="npz" (default when `filepath` ends with .npz, and WingLoop's default output):
        one compressed .npz; ModelStates as one float32 array (ASWING computes in single
        precision and prints 9 significant digits, which identify one float32 value, so
        float32 keeps ASWING's values exactly), every variable as its own array, names/units/
        metadata as JSON in "__meta__". About 3x smaller than the .json.gz (test case:
        1.7 vs 4.7 MB) and loaded without parsing text.
    fmt="json": V2 format, .json.gz (compress=True) or .json.

    Returns the path written. Keys starting with "_" (internal bookkeeping) are not saved.
    """
    import json, gzip, os
    if fmt is None:
        fmt = "npz" if filepath.endswith(".npz") else "json"
    for suffix in (".json.gz", ".json", ".npz"):
        if filepath.endswith(suffix):
            filepath = filepath[: -len(suffix)]
            break
    os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
    states = model_states_array(data_dict)

    if fmt == "npz":
        filepath += ".npz"
        names = list(data_dict.get("ModelVariables", {}).keys())
        meta = {
            "format": "WingLoop-npz-1",
            "ModelName": data_dict.get("ModelName"),
            "ModelParameters": _serialisable(data_dict.get("ModelParameters", {})),
            "Metadata": _serialisable(data_dict.get("Metadata", {})),
            "variables": [{"name": n,
                           "unit": data_dict["ModelVariables"][n].get("unit"),
                           "latex": data_dict["ModelVariables"][n].get("latex")} for n in names],
        }
        arrays = {"__meta__": np.array(json.dumps(meta, ensure_ascii=False)),
                  "ModelStates": states.astype(np.float32)}
        for i, n in enumerate(names):
            vals = data_dict["ModelVariables"][n]["values"]
            arr = np.asarray(vals)
            if arr.dtype == object:
                arr = np.asarray([np.nan if v is None else v for v in vals], dtype=float)
            arrays[f"v{i}"] = arr
        np.savez_compressed(filepath, **arrays)
    elif fmt == "json":
        filepath += ".json.gz" if compress else ".json"
        out = {k: v for k, v in data_dict.items() if not k.startswith("_")}
        out["ModelStates"] = states
        payload = json.dumps(_serialisable(out), indent=2, ensure_ascii=False).encode("utf-8")
        if compress:
            with gzip.open(filepath, "wb", compresslevel=6) as f:
                f.write(payload)
        else:
            with open(filepath, "wb") as f:
                f.write(payload)
    else:
        raise ValueError("fmt must be 'npz' or 'json'")

    print(f"[PyControl_IO] Saved -> {filepath}  ({len(data_dict.get('ModelVariables', {}))} variables, "
          f"{states.shape[0]} state snapshot(s), {os.path.getsize(filepath) / 1024:.1f} KB)")
    return filepath


def import_data_dict(filepath):
    """
    Load a file written by export_data_dict (.npz, .json.gz or .json; the extension may be
    omitted, then .npz, .json.gz and .json are tried in that order).

    Returns the V2 dictionary layout: ModelName, ModelParameters, ModelVariables
    {name: {"values": list, "unit", "latex"}}, ModelStates (2-D float64 array),
    plus Metadata when present.
    """
    import json, gzip, os
    stem = filepath
    for suffix in (".json.gz", ".json", ".npz"):
        if stem.endswith(suffix):
            stem = stem[: -len(suffix)]
            break
    candidates = [filepath] if os.path.exists(filepath) else []
    candidates += [stem + ".npz", stem + ".json.gz", stem + ".json"]
    resolved = next((p for p in candidates if os.path.isfile(p)), None)
    if resolved is None:
        raise FileNotFoundError(f"[PyControl_IO] Could not find file for: {filepath}\n  Tried: {candidates}")

    if resolved.endswith(".npz"):
        with np.load(resolved, allow_pickle=False) as z:
            meta = json.loads(str(z["__meta__"]))
            data_dict = {"ModelName": meta.get("ModelName"),
                         "ModelParameters": meta.get("ModelParameters", {}),
                         "Metadata": meta.get("Metadata", {}),
                         "ModelVariables": {},
                         "ModelStates": z["ModelStates"].astype(np.float64)}
            for i, v in enumerate(meta["variables"]):
                data_dict["ModelVariables"][v["name"]] = {"values": z[f"v{i}"].tolist(),
                                                          "unit": v.get("unit"), "latex": v.get("latex")}
    else:
        opener = gzip.open if resolved.endswith(".gz") else open
        with opener(resolved, "rb") as f:
            data_dict = json.loads(f.read().decode("utf-8"))
        raw = data_dict.get("ModelStates")
        if isinstance(raw, list) and raw:
            data_dict["ModelStates"] = np.array(raw, dtype=np.float64) if isinstance(raw[0], list) \
                else np.array(raw, dtype=np.float64)

    print(f"[PyControl_IO] Loaded <- {resolved}  ({len(data_dict.get('ModelVariables', {}))} variables, "
          f"{len(data_dict.get('ModelStates', []))} state snapshot(s))")
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

    data_test = read_aswing_file("test_files/test_output/test_output", data_imperial, rename_map)
    print_aswing_summary(data_test)
    export_data_dict(data_test,"data_test.json")
    export_data_dict(data_test,"data_test.json",compress=False)

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