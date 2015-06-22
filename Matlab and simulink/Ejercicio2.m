% 22.90 Automación Industrial  
% Trabajo Práctico Final
%
% Grupo:    51665 Vega, Juan Pablo
%           50615 Calvo, Clementina
%
% 17/11/2014


%% Ejercicio 2

clear all
close all
clc

initRobot();

% Change here for perturbed-unperturbed simulation
perturbModel = false;

initRobot();

if(perturbModel)
    probot = robot.perturb(0.8);
else
    probot = robot;
end;

T = 3;
fs = 200;
t = (0:(1/fs):T)';

%% Initial configuration

%Set here desired initial cartesian-space end effector configuration
xi = 0;     
yi = 1;

Ti = [eye(3),[xi;yi;0];[0 0 0 1]];
thetai = robot.ikine(Ti, [0, pi/2], [1 1 0 0 0 0]);

%% Wall

pWall = [[2,0,-2];[2,0,2];[0,2,2];[0,2,-2]];

Wall.Ke = 1000;

Wall.n = cross(pWall(1,:) - pWall(2,:), pWall(1,:) - pWall(3,:));
Wall.n = Wall.n/norm(Wall.n);
Wall.refPoint = pWall(1,:)';

%% Force controller

k = 0.01;
Kp = eye(robot.n) * k * (pi*fs)^2;
Kv = 2*sqrt(Kp); 
Kp = Kp/Wall.Ke;

%% Force 

steps = length(t);
Fd = 10;
FSetpoint = [Fd/sqrt(2)*ones(steps,1), Fd/sqrt(2)*ones(steps,1)]; % N
sim('my_robot_2');

%% Plots

figure('units','normalized','outerposition',[0 0 1 1]); % fullscreen

subplot(3,2,2);
plot(t, q);
title('Joint angles');
xlabel('t (s)');
ylabel('\theta (rad)');
legend('\theta_{1}', '\theta_{2}');

subplot(3,2,3);
plot(t, x);
title('Actual position - time');
xlabel('t (s)');
ylabel('X');
legend('Robot x', 'Robot y', 'Location', 'southeast');

subplot(3,2,4);
plot(x(:,1), x(:,2));
title('Actual position - XY');
xlabel('x');
ylabel('y');
legend('Actual', 'Location', 'northwest');

subplot(3,2,5);
plot(t, Fe, t, FSetpoint);
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

subplot(3,2,1);
wall = [[2,0,-2];[2,0,2];[0,2,2];[0,2,-2]];
fill3(wall(:,1), wall(:,2), wall(:,3), [1, 166/255, 166/255]);
hold all;
zoom(1.4);
quiver3(1, 1, 2, Wall.n(1), Wall.n(2), Wall.n(3), '-k') % plot normal vector
robot.plot(q, 'trail', '-');


