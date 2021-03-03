function cellInfo = BusAttitudeParameters(varargin) 
%BUSATTITUDEPARAMETERS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false; 
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
    error('Invalid input argument(s) encountered'); 
end 

cellInfo = { ... 
  { ... 
    'parameterBus', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', {... 
{'MC_SAMPLING_PERIOD', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ROLL_TC', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PITCH_TC', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ROLL_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ROLLRATE_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ROLLRATE_I', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ROLLRATE_D', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ROLLRATE_FF', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PITCH_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PITCHRATE_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PITCHRATE_I', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PITCHRATE_D', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PITCHRATE_FF', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAW_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAWRATE_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAWRATE_I', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAWRATE_D', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAWRATE_FF', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAW_FF', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ROLLRATE_MAX', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PITCHRATE_MAX', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAWRATE_MAX', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YAWRAUTO_MAX', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ACRO_R_MAX', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ACRO_P_MAX', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_ACRO_Y_MAX', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_RATT_TH', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_BREAK', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_SLOP', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VT_OPT_RECOV_EN', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VT_WV_YAWR_SCL', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_RR_INT_LIM', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_PR_INT_LIM', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_BREAK_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_BREAK_I', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_BREAK_D', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_RATE_P', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_RATE_I', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_TPA_RATE_D', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'VT_TYPE', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_BAT_SCALE_EN', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'MC_YR_INT_LIM', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 