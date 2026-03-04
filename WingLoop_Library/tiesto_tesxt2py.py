import re
import numpy as np


def is_likely_unit(u):
    """Quick filter to accept only realistic units and reject variable-like tokens."""
    if not u:
        return False
    if re.search(r'[/\-\^]', u):
        return True
    known = {"s", "m", "m/s", "m/s^2", "deg", "deg/s", "deg/s^2", "N", "N-m", "N/m^2",
             "kg/m^3", "m^2", "Kft", "ft", "slug", "dyn"}
    return u in known or u.lower() in known or u.upper() in known


def initialize_data_dict(requested, rename_map=None, units=None, latex=None):
    if rename_map is None:
        rename_map = {}
    if units is None:
        units = {}
    if latex is None:
        latex = {}

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

def read_aswing_file(filepath, data_dict, rename_map=None):
    if rename_map is None:
        rename_map = {}

    # ------------------------------------------------------------------
    # Build exact-match regex from all requested variables
    # ------------------------------------------------------------------
    raw_names = list(rename_map.keys())
    for internal in list(data_dict["ModelVariables"].keys()):
        if internal not in rename_map.values():
            raw_names.append(internal)

    all_raw = set(raw_names)
    sorted_raw = sorted(all_raw, key=len, reverse=True)   # longest first
    name_alt = '|'.join(re.escape(name) for name in sorted_raw)

    first_words = {name.split()[0] for name in all_raw if name}
    sorted_first = sorted(first_words, key=len, reverse=True)
    first_alt = '|'.join(re.escape(w) for w in sorted_first)

    pattern = re.compile(rf"""
        (?<![A-Za-z0-9])             # <<< NEW: prevents matching "e" inside "Engine", "Peng", "Density"...
        ({name_alt})                 # exact raw variable name as it appears in the file
        \s*:\s*
        ([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)   # numeric value
        (?:\s+(?!(?:{first_alt})(?=[\s:]|$))([^\s:]+))?   # unit (only if not start of next var)
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
            var_name = match.group(1).strip()          # raw name from file
            value = float(match.group(2))
            unit_candidate = match.group(3)

            internal_name = rename_map.get(var_name, var_name)

            if internal_name in data_dict["ModelVariables"]:
                data_dict["ModelVariables"][internal_name]["values"].append(value)

                if (data_dict["ModelVariables"][internal_name]["unit"] is None and
                        unit_candidate is not None and
                        is_likely_unit(unit_candidate)):
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

    # Control
    "Flap 1",
    "Flap 2",
    "Flap 3",
    "Flap 4",

    # Engine (optional)
    "Peng 1","Peng 2",

    # Convergence (we will treat specially)
    "IsConverged",
    
    "Op.Point",
    "altitude"
]



rename_map = {
    "earth X": "earthX",
    "earth Y": "earthY",
    "earth Z": "earthZ",
    "Elev.": "Pitch",
    "Flap 2": "F2"
}

units = {
#    "Time": "s",
#    "earthX": "m",
#    "Velocity": "m/s",
#    "Wy": "deg/s"
}

latex = {
    "Time": r"$t$",
    "earthX": r"$X_E$",
    "Velocity": r"$V$"
}

adimensional_vars = {
    "CL", "CD", "Cm", "Cn'",
    "Mach", "MachPG", "Load Fac",
    "Flap 1", "Flap 2", "Flap 3", "Flap 4",
    "Peng 1", "Peng 2","e"
}

data_metric = initialize_data_dict(requested, rename_map, units, latex)   # ← added rename_map here

data_metric = read_aswing_file("testfiles/ASWING_test_output_metric", data_metric, rename_map)
print_aswing_summary(data_metric)

data_imperial = initialize_data_dict(requested, rename_map, units, latex)   # ← added rename_map here
data_imperial = read_aswing_file("testfiles/ASWING_test_output_imperial", data_imperial, rename_map)
print_aswing_summary(data_imperial)