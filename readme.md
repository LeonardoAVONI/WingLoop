The 01_Control code is made to couple ASWING with Python/Matlab, for them to perform control on the aircraft simulated in ASWING

The exact behavior of how the code works is detailed in the correspondent report. The general idea is to make ASWING run N-iteration time-transient simulations in groups of K iterations:
	-using the control scheme defined in matlab or python, the next instructions for 
	the control surfaces+engines are written to a text document
	-oper>X is launched with K iterations, that use the previously defined aileron+engine 
	instructions and compute the aircraft state after >K iterations
	-the state of the aircraft is written to a text document
	-the state is read by the python/matlab control strategy, that then figures out what 
	the next aileron+engine instruction should be for the aircraft to fulfil the instructions
Those steps are iterated until the N iterations are done
