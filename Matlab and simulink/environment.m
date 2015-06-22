function alpha(block)

setup(block);

function setup(block)

%% INPUTS
block.NumInputPorts  = 1;

block.InputPort(1).Complexity   = 'Real';
block.InputPort(1).Dimensions   = 2;

%% PARAMETERS
block.NumDialogPrms = 2;

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
block.RegBlockMethod('Outputs', @Output);    % Register methods


function Output(block)
theta = block.InputPort(1).Data';
robot = block.DialogPrm(1).Data;    
Wall  = block.DialogPrm(2).Data;

x_ee = transl(robot.fkine(theta));

d = Wall.n * (x_ee - Wall.refPoint);

if (d < 0)
    Fe = (Wall.Ke * d) * Wall.n;
else
    Fe = zeros(robot.n,1)';
end
    
block.OutputPort(1).Data = Fe(:,1:2)';