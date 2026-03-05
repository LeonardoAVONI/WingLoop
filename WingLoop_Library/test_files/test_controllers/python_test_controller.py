
def python_test_controller(instantaneous_state, Dt):
    """  
    Very simple python control file:
        takes the input state
        takes the first 4 elements of the state and associates them with flap deflections F1 to F4
        takes the last 2 elements of defaultoutput and associates them with E1 and E2
    This controller is to be used with an aircraft with 6 control inputs
    
    """
    # default output
    defaultoutput = [0,0,0,0,10,10]

    # Sending the final instructions
    output = {}
    output["F1"]=  instantaneous_state[0]
    output["F2"]=  instantaneous_state[1]
    output["F3"]=  instantaneous_state[2]
    output["F4"]=  instantaneous_state[3]
    
    # forcing the engine output
    output["E1"]= defaultoutput[4]
    output["E2"]= defaultoutput[5]
    
    return output
    
    
    