function Arm(block)

% SETUP
setup(block);

end

function setup(block)

% PARAMETERS
block.NumDialogPrms  = 1;
robot = block.DialogPrm(1).Data;

% INPUT
block.NumInputPorts  = 3;
block.SetPreCompInpPortInfoToDynamic;
block.InputPort(1).Dimensions = robot.n;
block.InputPort(2).Dimensions = robot.n;
block.InputPort(3).Dimensions = robot.n;

% OUTPUT
block.NumOutputPorts = 1;
block.SetPreCompOutPortInfoToDynamic;
block.OutputPort(1).Dimensions = robot.n;

% SAMPLE TIME
block.SampleTimes = [-1 0];

% "COMPILED" (ACCELERATED) CODE
block.SetAccelRunOnTLC(true);

% METHODS
block.RegBlockMethod('Outputs', @foo);     % Required

end

function foo(block)

% INPUTS
tau    = block.InputPort(1).Data';
q      = block.InputPort(2).Data';
q_dot  = block.InputPort(3).Data';

% PARAMETERS
robot  = block.DialogPrm(1).Data;

% FOO
q_ddot = robot.accel(q, q_dot, tau);

% OUTPUT
block.OutputPort(1).Data = q_ddot';

end
