% 22.90 Automación Industrial  
% Trabajo Práctico Final
%
% Grupo:    51665 Vega, Juan Pablo
%           50615 Calvo, Clementina
%
% 17/11/2014


%% Ejercicio 1

clear all
close all
clc

% Change here for perturbed-unperturbed simulation
perturbModel = false;

initRobot();

if(perturbModel)
    probot = robot.perturb(0.8);
else
    probot = robot;
end;

T = 1;
fs = 200;
t = (0:(1/fs):T)';

k = 0.01;
Kp = eye(robot.n) * k * (pi*fs)^2;
Kv = 2*sqrt(Kp); 

%% Initial configuration

% Set here desired initial cartesian-space end effector configuration
xi = 0.5;     
yi = 0;

Ti = [eye(3),[xi;yi;0];[0 0 0 1]];
thetai = robot.ikine(Ti, [0, pi/2], [1 1 0 0 0 0]);

%% Trajectory

% Set here desired final cartesian-space end effector configuration
xf = 1;
yf = 1;

steps = length(t);

xSetpoint = [lspb(xi, xf, steps), lspb(yi, yf, steps)];
sim('my_robot_1');

%% Plots

figure('units','normalized','outerposition',[0 0 1 1]); % fullscreen

subplot(2,2,2);
plot(t, q);
title('Joint angles');
xlabel('t (s)');
ylabel('\theta (rad)');
legend('\theta_{1}', '\theta_{2}');

subplot(2,2,3);
plot(t, x, t, xSetpoint, t(end), [xf, yf], 'x');
title('Actual position v. setpoint - time');
xlabel('t (s)');
ylabel('X');
legend('Robot x', 'Robot y', 'Setpoint x', 'Setpoint y', 'Location', 'southeast');

subplot(2,2,4);
plot(x(:,1), x(:,2), xSetpoint(:,1), xSetpoint(:,2), xf, yf, 'x');
title('Actual position v. setpoint - XY');
xlabel('x');
ylabel('y');
legend('Actual', 'Setpoint', 'Location', 'southeast');

%% Animation

subplot(2,2,1);
wall = [[2,0,-2];[2,0,10];[0,2,10];[0,2,-2]];
fill3(wall(:,1), wall(:,2), wall(:,3), [1, 166/255, 166/255]);
hold all;
zoom(1.4)
robot.plot(q, 'trail', '-');


