% 22.90 Automación Industrial  
% Trabajo Práctico Final
%
% Grupo:    51665 Vega, Juan Pablo
%           50615 Calvo, Clementina
%
% 17/11/2014


%% Ejercicio 3

clear all
close all
clc

perturbModel = true;

initRobot();

if(perturbModel)
    probot = robot.perturb(0.8);
else
    probot = robot;
end

T = 3;
fs = 200;
t = (1/fs:(1/fs):T)';
steps = length(t);

%% Initial configuration

%Set here desired initial cartesian-space end effector configuration
xi = .5;     
yi = .5;

Ti = [eye(3),[xi;yi;0];[0 0 0 1]];
thetai = robot.ikine(Ti, [0, pi/2], [1 1 0 0 0 0]);

%% Wall
        %righ down %right up %left down %left up %middle
pWall = [[2,0,-2];[2,0,10];[0,2,-2]; [0,2,10]; [1,1,0]]; 

Wall.Ke = 1000;

Wall.n = cross(pWall(1,:) - pWall(2,:), pWall(1,:) - pWall(3,:));
Wall.n = Wall.n/norm(Wall.n);
Wall.refPoint = pWall(end,:)';
Wall.angle = atan(((pWall(3,2)-pWall(1,2)))/(pWall(1,1)-pWall(3,1))); % atan(deltaY/deltaX)
Wall.rot = rotz(pi/2-Wall.angle);
Wall.rot = Wall.rot(1:2,1:2);

%% Position controller

Kpp = eye(robot.n) * 0.1 * (pi*fs)^2;
Kvp = 2*sqrt(Kpp); 

%% Force controller

Kpf = eye(robot.n) * 0.1 * (pi*fs)^2;
Kvf = 2*sqrt(Kpf); 
Kpf = Kpf/Wall.Ke;

%% Hybrid controller

%% Trajectory

% Middle point: make the trajectory go through the wall

xf = Wall.refPoint(1);
yf = Wall.refPoint(2);

xSetpoint = [lspb(xi, xf, steps/2), lspb(yi, yf, steps/2)];

xi = Wall.refPoint(1)+.01;   % so the controller recognizes the wall
yi = Wall.refPoint(2)+.01;

% Set here desired final cartesian-space end effector configuration
xf = 1.7;   %so that it doesn't reach the singularity in (2,0).
yf = 0.3;

xSetpoint = [xSetpoint; lspb(xi, xf, steps/2), lspb(yi, yf, steps/2)];

%% Force 

Fd = 10; 
FSetpoint = [Fd/sqrt(2)*ones(steps,1), Fd/sqrt(2)*ones(steps,1)]; % N


%% Run

sim('my_robot_3_v2');

%% Plots

figure('units','normalized','outerposition',[0 0 1 1]); % fullscreen

subplot(3,2,2);
plot(t, q(1:end-1,:));
title('Joint angles');
xlabel('t (s)');
ylabel('\theta (rad)');
legend('\theta_{1}', '\theta_{2}');

subplot(3,2,4);
plot(t, x(1:end-1,:), t, xSetpoint);
title('Actual position v. setpoint - time');
xlabel('t (s)');
ylabel('X');
legend('Robot x', 'Robot y', 'Setpoint x', 'Setpoint y', 'Location', 'southeast');

plot(x(:,1), x(:,2), xSetpoint(:,1), xSetpoint(:,2));
title('Actual position v. setpoint - XY');
xlabel('x');
ylabel('y');
legend('Actual', 'Setpoint', 'Location', 'southwest');

subplot(3,2,5);
plot(t, Fe(1:end-1,:), t, FSetpoint);
title('Actual force v. setpoint - time');
xlabel('t (s)');
ylabel('F (N)');
legend('Robot F_{x}', 'Robot F_{y}', 'Setpoint F_{x}', 'Setpoint F_{y}', 'Location', 'southeast');

subplot(3,2,6);
plot(Fe(:,1), Fe(:,2), FSetpoint(:,1), FSetpoint(:,2), 'x');
title('Actual force v. setpoint - XY');
xlabel('F_{x}');
ylabel('F_{y}');
legend('Actual', 'Setpoint', 'Location', 'northwest');



%% Animation

subplot(2,2,1);
wall = [[2,0,-2];[2,0,2];[0,2,2];[0,2,-2]];
fill3(wall(:,1), wall(:,2), wall(:,3), [1, 166/255, 166/255]);
hold all;
quiver3(1, 1, 2, Wall.n(1), Wall.n(2), Wall.n(3))
robot.plot(q, 'trail', '-');


