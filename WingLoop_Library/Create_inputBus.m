% Example: a struct with 1945 fields (field names can be S1, S2, ... S1945)
stateStruct = struct();
for i = 1:1945
    fieldName = ['S' num2str(i)];
    stateStruct.(fieldName) = 0;  % initial value
end

% Create a Bus from this struct
Simulink.Bus.createObject(stateStruct);