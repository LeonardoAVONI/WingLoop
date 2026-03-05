"""
====================================================================================
Text2Python_Library, Package Version

Author: Leonardo AVONI
Date: 25/10/2024
Email: avonileonardo@gmail.com

Last modified:  29/05/2025

====================================================================================

Description:
Within the WingLoop_Library, this function provides functions used mainly by any WingLoop 
instance. Refer to WingLoop th check which function is used where

Sometimes, if the "input" files for WingLoop are badly written, they are not recognized by ASWING    
====================================================================================
"""

import numpy as np
import re


def initialize_data_dict(requested, rename_map=None, latex=None):
    if rename_map is None:
        rename_map = {}
    if latex is None:
        latex = {}
    units = {}

    data = {
        "ModelName": None,
        "States": None,
        "ModelVariables": {}
    }

    # Use the final (clean) internal name as dict key for consistency with units/latex
    for raw_var in requested:
        internal_var = rename_map.get(raw_var, raw_var)
        data["ModelVariables"][internal_var] = {
            "values": [],
            "unit": units.get(internal_var) or units.get(raw_var),
            "latex": latex.get(internal_var) or latex.get(raw_var)
        }

    return data

def initialize_control_dict(control_elements, default_value=0.0):
    """
    Creates a control dictionary with all requested control elements set to a default value.
    
    Example:
        control = initialize_control_dict(["F1", "F2", "F3", "F4", "E1", "E2"])
        # → {'F1': 0.0, 'F2': 0.0, 'F3': 0.0, 'F4': 0.0, 'E1': 0.0, 'E2': 0.0}
    """
    return {elem: default_value for elem in control_elements}

def read_aswing_file(filepath, data_dict, rename_map=None):
    if rename_map is None:
        rename_map = {}

    # ------------------------------------------------------------------
    # Collect ALL raw names exactly as they appear in the file
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Final ultra-robust pattern
    # ------------------------------------------------------------------
    pattern = re.compile(rf"""
        (?<![A-Za-z0-9])                # word boundary
        ({name_alt})                    # exact raw variable
        \s*:\s*
        ([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)   # value
        (?:\s+                          # optional unit
            (?! (?:{first_alt})(?=[\s:]|$))   # NOT start of any known variable
            ([^\s:]+)                   # the unit token
            (?= \s (?! :) | $ )         # followed by space-not-colon OR end-of-line
        )?
    """, re.VERBOSE)

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
        if "ENDEDPRINTINGSTATES" in stripped:
            recording_states = False
            data_dict["States"] = np.array(states)
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

    states = data_dict.get("States")
    if states is not None:
        print(f"States length   : {len(states)}")
        print(f"States preview  : {states[:5]} ... {states[-5:]}")
    else:
        print("States          : None")

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
    Writes a simple ASWING time-history control file using the provided control dictionary.
    
    Parameters:
        filename : str, path to the output control file (e.g. "controls.dat")
        control  : dict, must contain all required control keys ("F1", "F2", "E1", etc.)
                   Keys determine both the column order and the values.
    
    Raises:
        KeyError if any expected control is missing (but since we assume the dict is complete,
        this is considered a programming error upstream).
    """
    # The column order = insertion order of keys in the dict (Python 3.7+)
    control_elements = list(control.keys())
    
    header = " time " + " ".join(control_elements)+" "+ "\n"
    
    # Convert values to strings
    values = [str(value) for value in control.values()]
    
    line1 = " 0.0\t" + "\t".join(values) + "\n"
    line2 = " 1000.0\t" + "\t".join(values)
    
    content = header + line1 + line2
    
    with open(filename, "w") as f:
        f.write(content)

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
        data_metric = read_aswing_file("testfiles/ASWING_test_output_metric", data_metric, rename_map)

    data_imperial = initialize_data_dict(requested, rename_map, latex)   # ← added rename_map here
    data_imperial = read_aswing_file("testfiles/ASWING_test_output_imperial", data_imperial, rename_map)
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