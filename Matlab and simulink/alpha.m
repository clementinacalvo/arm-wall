function alpha(block)

setup(block);

function setup(block)

%% INPUTS
block.NumInputPorts  = 2;

block.InputPort(1).Complexity   = 'Real';
block.InputPort(1).Dimensions   = 2;
block.InputPort(2).Complexity   = 'Real';
block.InputPort(2).Dimensions   = 2;


%% PARAMETERS
block.NumDialogPrms = 1;

%% OUTPUTS
block.NumOutputPorts = 1;

block.OutputPort(1).Complexity   = 'Real';
block.OutputPort(1).Dimensions   = 2;

%% SAMPLE TIME
block.SampleTimes = [-1 0];   %% Set block sample time to inherited

%% OTHER
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Set the block simStateComliance to default (i.e., same as a built-in block)
block.SimStateCompliance = 'DefaultSimState';
block.SetAccelRunOnTLC(true);  %% Run accelerator on TLC

%% METHODS
block.RegBlockMethod('Outputs',                 @Output);    % Register methods


function Output(block)
xDotDot = block.InputPort(1).Data';
theta   = block.InputPort(2).Data';
robot   = block.DialogPrm(1).Data';

J       = robot.jacob0(theta, 'trans');
alpha = robot.inertia(theta) / J(1:2,:) * xDotDot';

block.OutputPort(1).Data = alpha';