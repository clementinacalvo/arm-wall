function beta(block)

setup(block);

function setup(block)
%% Register number of input and output ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 1;
%% Register number of dialog parameters
block.NumDialogPrms = 1;

%% Setup functional port properties to dynamically inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).Complexity   = 'Real';
block.InputPort(1).Dimensions   = 2;
block.InputPort(2).Complexity   = 'Real';
block.InputPort(2).Dimensions   = 2;

block.OutputPort(1).Complexity   = 'Real';
block.OutputPort(1).Dimensions   = 2;

% block.InputPort(1).DirectFeedthrough = true;
block.SampleTimes = [-1 0];   %% Set block sample time to inherited

%% Set the block simStateComliance to default (i.e., same as a built-in block)
block.SimStateCompliance = 'DefaultSimState';
block.SetAccelRunOnTLC(true);  %% Run accelerator on TLC
block.RegBlockMethod('Outputs',                 @Output);    % Register methods


function Output(block)
theta      = block.InputPort(1).Data';
thetaDot   = block.InputPort(2).Data';
robot      = block.DialogPrm(1).Data;

M = robot.inertia(theta);
J = robot.jacob0(theta, 'trans');
JDot = robot.jacob_dot(theta, thetaDot);

V = robot.coriolis(theta, thetaDot) * thetaDot' - M / J(1:2,:) * JDot(1:2);
G = robot.gravload(theta);
B = robot.friction(thetaDot);

beta = V' + G - B;

block.OutputPort(1).Data = beta;