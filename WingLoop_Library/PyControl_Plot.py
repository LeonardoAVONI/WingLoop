import numpy as np

"""  
Final graph and during-simulation graph

What I want on the graph:
    Model variables to plot according to a user-provided list
    One live-updating version, and one final version.
    The functions would receive the dictionary at each timestep
    

The following dictionary structure as input

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