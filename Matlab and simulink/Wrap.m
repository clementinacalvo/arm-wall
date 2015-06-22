function Wrap(block)

% SETUP
setup(block);

end

function setup(block)

% PARAMETERS
block.NumDialogPrms  = 1;
robot = block.DialogPrm(1).Data;

% INPUT
block.NumInputPorts  = 1;
block.SetPreCompInpPortInfoToDynamic;
block.InputPort(1).Dimensions = robot.n;

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

% INPUTS
q  = block.InputPort(1).Data';

% FOO
qw = wrapToPi(q);

% OUTPUT
block.OutputPort(1).Data = qw;

end
