function Jacobian(block)

% SETUP
setup(block);

end

function setup(block)

% PARAMETERS
block.NumDialogPrms  = 1;
robot = block.DialogPrm(1).Data;

% INPUT
block.NumInputPorts  = 2;
block.SetPreCompInpPortInfoToDynamic;
block.InputPort(1).Dimensions = robot.n;
block.InputPort(2).Dimensions = robot.n;

% OUTPUT
block.NumOutputPorts = 1;
block.SetPreCompOutPortInfoToDynamic;
block.OutputPort(1).Dimensions = robot.n;

% SAMPLE TIME
block.SampleTimes = [-1 0];

% "COMPILED" (ACCELERATED) CODE
block.SetAccelRunOnTLC(true);

% METHODS
block.RegBlockMethod('Outputs', @foo);

end

function foo(block)

% PARAMETERS
robot = block.DialogPrm(1).Data;

% INPUTS
q = block.InputPort(1).Data';
q_dot = block.InputPort(2).Data';

% FOO
Jo = robot.jacob0(q, 'trans');
x_dot = Jo*q_dot';

% OUTPUT
block.OutputPort(1).Data = (x_dot(1:2))';

end
