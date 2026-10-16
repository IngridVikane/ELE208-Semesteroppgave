%% 6DOF Robot Arm for Palletizing

clc;
clear;

syms q1 q2 q3 q4 q5 q6 real
syms L1 L2 L3 L4 L5 L6 real


import ETS3.*

%% Lenkelengder - usikker på lengdene
robot.L1 = L1;
robot.L2 = L2;
robot.L3 = L3;
robot.L4 = L4;
robot.L5 = L5;
robot.L6 = L6; 

DHmatrix = @(p) [ ...
    cos(p.theta), -sin(p.theta)*cos(p.alpha),  sin(p.theta)*sin(p.alpha), p.a*cos(p.theta);
    sin(p.theta),  cos(p.theta)*cos(p.alpha), -cos(p.theta)*sin(p.alpha), p.a*sin(p.theta);
    0,           sin(p.alpha),             cos(p.alpha),            p.d;
    0,           0,                       0,                    1];


%% Robot model
robot.DH(1) = struct('type','R', 'a',0, 'alpha',pi/2, 'd',L1, 'theta',q1);
robot.DH(2) = struct('type','R', 'a',L2, 'alpha',0, 'd',0, 'theta',q2+pi/2);
robot.DH(3) = struct('type','R', 'a',L3, 'alpha',0, 'd',0, 'theta',q3);
robot.DH(4) = struct('type','R', 'a',0, 'alpha',-pi/2, 'd',L4, 'theta',q4-pi/2);
robot.DH(5) = struct('type','R', 'a',0, 'alpha',pi/2, 'd',L5, 'theta',q5);
robot.DH(6) = struct('type','R', 'a',0, 'alpha',0, 'd',L6, 'theta',q6);

%% Transformasjon
T01 = DHmatrix(robot.DH(1))
T12 = DHmatrix(robot.DH(2))
T23 = DHmatrix(robot.DH(3))
T34 = DHmatrix(robot.DH(4))
T45 = DHmatrix(robot.DH(5))
T56 = DHmatrix(robot.DH(6))

%% Sizing the robot (Legg inn faktiske lenkelengder)
robot.L1 = 0.35;
robot.L2 = 0.3;
robot.L3  = 0.3;
robot.L4  = 0.3;
robot.L5  = 0.3;
robot.L6  = 0.3;

%% Defining Palletizing robot design

%% Create SerialLink model

L(1) = Link('revolute', ...
    'd', robot.L1, ...
    'a', 0, ...
    'alpha', pi/2);

L(2) = Link('revolute', ...
    'd', 0, ...
    'a', robot.L2, ...
    'alpha', 0, ...
    'offset', pi/2);

L(3) = Link('revolute', ...
    'd', 0, ...
    'a', robot.L3, ...
    'alpha', 0);

L(4) = Link('revolute', ...
    'd', robot.L4, ...
    'a', 0, ...
    'alpha', -pi/2, ...
    'offset', -pi/2);

L(5) = Link('revolute', ...
    'd', robot.L5, ...
    'a', 0, ...
    'alpha', pi/2);

L(6) = Link('revolute', ...
    'd', robot.L6, ...
    'a', 0, ...
    'alpha', 0);


%% Create robot
Palletizing = SerialLink(L, 'name', 'Palletizing Robot');

q = [0 0 0 0 0 0];
T = Palletizing.fkine(q)

% Joint limits
Palletizing.qlim = [ ...
    -pi         pi;          % Joint 1: revolute
    -pi         pi;          % Joint 2: revolute
    deg2rad([-160 160]);     % Joint 3: revolute
    -pi         pi;          % Joint 4: revolute
    -pi         pi;          % Joint 5: revolute
    -pi         pi];         % Joint 6: revolute

q0 = [0 0 0 0 0 0];

figure
Palletizing.plot(q0)
Palletizing.teach