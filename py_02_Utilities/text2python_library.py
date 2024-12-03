"""
====================================================================================
test2python_library, Version 2

Author: Leonardo AVONI
Date: 25/10/2024
Email: avonileonardo@gmail.com

Last modified:  03/12/2024

====================================================================================

Description:
This Python class provides functions helping any instance of Aswing_Director_Library
to read and write data from the text documents used within ASWING

Features:
- extract_value: extract state variables from the output document produced by ASWING
- text2python: converts string variables
- scientific_to_decimal and convert_extracted_values: help having a variable 
    format acceptable by MATLAB
- python2text: from python variables: writes the new Open-Loop forcing text file

Comments:
- the python2text has been checked in order to ensure that its output is properly 
    recognized by ASWING

Changelog:
- Version 1 (25/10/2024): first version of the code
- Version 2 (28/10/2024): modified python2text (unrecognized ASWING input)
- Version 3 (29/10/2024): various functions were added, trying to read as fast 
    as possible the text files from ASWING
        # text2python_lines, slow
        # text2python_main, fastest for the moment
        # text2python_mmap, second faster
        # text2python_mmap_Nlines, problematic
- Version 4 (30/10/2024): only text2python_main function was kept, 
    the others were commented
- Version 5 (03/12/2024): made better comments for the code to be put on Github

Possible improvements:
    Making the class independent to the type of UAV used 
    (for the moment the text2python_main>fields are specified, and also the python2text>F1,F2,F3...)

====================================================================================
"""

import re
import time
import numpy as np
import os
import mmap
import warnings
from itertools import islice


def extract_value(label, text):
    """
    Function that extracts the numbers that come after the specified labels, from a certain text i.e. multiline string
    """
    
    # Pattern to find the value after the label, allowing for spaces and units
    pattern = rf"{label}\s*:\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    match = re.search(pattern, text)
    if match:
        return match.group(1)
    else:
        return None

def seconds2hms(seconds):
    """
    Converts a time expressed in seconds -typically obtained using time.time()- to a time in h, m, s
    """
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    remaining_seconds = np.round(seconds % 60)
    return hours, minutes, remaining_seconds

def text2python_main(name):
    """
    opens a text document called "name", then extracts from it all the values written in "fields" from such document
    (we assume the document to be created from OPER)
    then, the values are extracted and stored in a dictionary called "extracted_values"
    """
    with open(name, "r") as document:
        text = document.read()
    #print(text)

    # List of fields to extract
    fields = ['Time','earth X','earth Y','earth Z','Heading', 'Elev.', 'Bank','Alpha','Beta','Velocity','Flap 2']

    # Extract the values for each field
    extracted_values = {field: extract_value(field, text) for field in fields}

    # Rename keys using a mapping
    key_mapping = {
        'earth X': 'earthX',
        'earth Y': 'earthY',
        'earth Z': 'earthZ',
        'Elev.': 'Pitch',
        'Flap 2': 'F2',
    }

    # Update the dictionary with the new key names
    for old_key, new_key in key_mapping.items():
        if old_key in extracted_values:
            extracted_values[new_key] = extracted_values.pop(old_key)

    extracted_values=convert_extracted_values(extracted_values)

    return extracted_values

def scientific_to_decimal(sci_str):
    """ 
    Convert the scientific notation string to a regular float and format it as a string
    This function is used by "convert_extracted_values"
    We had to convert from 1E5 to 100000 because otherwise the numbers could not be used by Matlab
    """
    num = float(sci_str)
    return f"{num:.10f}".rstrip("0").rstrip(".")

def convert_extracted_values(extracted_values):
    """
    Takes the input "extracted values" dictionary, and for each value inside it does the following
        -converts it to non-scientific notation data (in case it needs to be used by matlab later)
        -in case some variables take value None, then it usually means that the iterations in ASWING 
        did not provid meaningful results, thus leading to non-computable data. In that case, a warning 
        is printed and an artificial value of 1E10 is used instead
    """
    # Iterate over each key-value pair in extracted_values
    for key, value in extracted_values.items():
        # Check if the item contains scientific notation, and convert if necessary
        if isinstance(value, list):  # Check for list values (multiple items)
            extracted_values[key] = [scientific_to_decimal(str(val)) if 'E' in str(val) else val for val in value]
        elif 'E' in str(value):  # Single value, convert if scientific notation is found
            extracted_values[key] = scientific_to_decimal(str(value))
        
        if extracted_values[key] is None:
            warnings.warn('INEXISTENT ASWING DATA: probable Newton iterations convergence problems')
            extracted_values[key] = 10000000000
        else:
            extracted_values[key] = float(extracted_values[key]) #convert to float
    return extracted_values


def python2text(filename, control):
    """ 
    Writes the informations for the flaps and engine to use
    In this case, it writes F1, F2, F3, F4 and E1
    It should be possible to make this function generalizable and 
    just create the file depending on the entries on the "control" dictionary
    
    Also, when writing the document, it is important to write it correctly, else 
    it will not be red properly by ASWING
    """

    Flap1 = str(control["F1"])
    Flap2 = str(control["F2"])
    Flap3 = str(control["F3"])
    Flap4 = str(control["F4"])
    Engine = str(control["E1"])

    # Build the content in a single string
    content = (
        " time  F1  F2  F3  F4  E1\n"
        f" 0.0\t{Flap1}\t{Flap2}\t{Flap3}\t{Flap4}\t{Engine}\n"
        f" 1000.0\t{Flap1}\t{Flap2}\t{Flap3}\t{Flap4}\t{Engine}"
    )

    # Write the content to the file at once
    with open(filename, "w") as file:
        file.write(content)
    
    # possible check to see if the file has finished writing, but we noticed the file
    # is written in one single time, so its not really useful

if __name__=="__main__":
    a = scientific_to_decimal(1E5)
    print(a+"ciao")

"""

def python2text_old(filename, Flap1, Flap2, Flap3, Flap4,):
    # Build the content in a single string, old, deprecated version
    content = (
        "time\tF1\tF2\tF3\tF4\n"
        "#\n"
        "*1.\t1.\t1.\t1.\t1.\n"
        "+0.\t0.\t0.\t0.\t0.\n"
        "#\n"
        f"0.0\t{Flap1}\t{Flap2}\t{Flap3}\t{Flap4}\n"
        f"1000.0\t{Flap1}\t{Flap2}\t{Flap3}\t{Flap4}\n"
    )

    # Write the content to the file at once
    with open(filename, "w") as file:
        file.write(content) #maybe use file.writelines(content)

def text2python_fromdoc(text):
    # List of fields to extract
    fields = ['Time','Heading', 'Elev.', 'Bank' , 'Velocity', 'Lift', 'Weight', 'Mach', 'CL', 'CD']

    # Extract the values for each field
    extracted_values = {field: extract_value(field, text) for field in fields}
    #print(extracted_values)

    extracted_values=convert_extracted_values(extracted_values)

    #print(extracted_values["Time"])

    return extracted_values["Elev."], extracted_values["Bank"], extracted_values["Heading"]


def text2python_mmap(name):
    # Sample input document as a multiline string

    with open(name, "r") as document:
        with mmap.mmap(document.fileno(), 0, access=mmap.ACCESS_READ) as mmapped_file:
            text = mmapped_file.read().decode('utf-8')

    # List of fields to extract
    fields = ['Heading', 'Elev.', 'Bank' , 'Velocity', 'Lift', 'Weight', 'Mach', 'CL', 'CD']

    # Extract the values for each field
    extracted_values = {field: extract_value(field, text) for field in fields}

    extracted_values=convert_extracted_values(extracted_values)

    return extracted_values["Elev."], extracted_values["Bank"], extracted_values["Heading"]

def text2python_lines(name,N=31):
    # Sample input document as a multiline string

    with open(name, "r") as document:
        text = ''.join(islice(document, N))

    # List of fields to extract
    fields = ['Heading', 'Elev.', 'Bank' , 'Velocity', 'Lift', 'Weight', 'Mach', 'CL', 'CD']

    # Extract the values for each field
    extracted_values = {field: extract_value(field, text) for field in fields}

    extracted_values=convert_extracted_values(extracted_values)

    return extracted_values["Elev."], extracted_values["Bank"], extracted_values["Heading"]

def text2python_mmap_Nlines(name,N=31):
    # Sample input document as a multiline string

    text = []
    with open(name, "r") as document:
        with mmap.mmap(document.fileno(), 0, access=mmap.ACCESS_READ) as mmapped_file:
            start = 0
            for _ in range(N):
                end = mmapped_file.find(b'\n', start)
                if end == -1:
                    # End of file reached before N lines
                    break
                text.append(mmapped_file[start:end + 1].decode('utf-8'))
                start = end + 1

    # List of fields to extract
    fields = ['Heading', 'Elev.', 'Bank' , 'Velocity', 'Lift', 'Weight', 'Mach', 'CL', 'CD']

    # Extract the values for each field
    extracted_values = {field: extract_value(field, text) for field in fields}

    extracted_values=convert_extracted_values(extracted_values)

    return extracted_values["Elev."], extracted_values["Bank"], extracted_values["Heading"]
"""