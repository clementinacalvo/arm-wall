function hybridController(block)

setup(block);

function setup(block)

%% INPUTS
block.NumInputPorts  = 2;

block.InputPort(1).Complexity   = 'Real';
block.InputPort(1).Dimensions   = 2;

block.InputPort(2).Complexity   = 'Real';
block.InputPort(2).Dimensions   = 2;

%% PARAMETERS
% block.NumDialogPrms = 1;

%% OUTPUTS
block.NumOutputPorts = 1;
block.OutputPort(1).Dimensions = 2;

%% SAMPLE TIME
block.SampleTimes = [-1 0];   %% Set block sample time to inherited

%% OTHER
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Set the block simStateComliance to default (i.e., same as a built-in block)
block.SimStateCompliance = 'DefaultSimState';
block.SetAccelRunOnTLC(true);  %% Run accelerator on TLC

%% METHODS
block.RegBlockMethod('Outputs', @Output);    % Register methods


function Output(block)
Fe = block.InputPort(1).Data';
FSetpoint= block.InputPort(2).Data';

% S is used for position
% S' is used for force
% where S' = diag(~diag(S))

histeresis = 1; % N

if Fe(2) <= 0
    %position control in both axes
    S = [1, 1];
elseif Fe(2) < (FSetpoint(2)-histeresis)
    %force control
    S = [0, 0];
else
    %mixed control
	S = [1, 0];
end

block.OutputPort(1).Data = S;   % only the diagonal is transmitted for simplicity