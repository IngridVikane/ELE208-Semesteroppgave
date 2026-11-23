%% Denne filen lager selve robotmodellen, "Palletizing Robot" ved hjelp av 
%% Peter Corke's Robotics Toolbox. 

% % 
%    Copyright:     HVL
%    Project:	    ELE308 - Robotmanipulatorer
%    Author:        Ingrid Vikane
%    Date created:  Høst 2026
%    Input:       
%           
%    Output: 'Palletizing Robot'         
%---------------------------------------------------------------------%
%%
% Note: Dette er en 6DOF robotarm som skal benyttes til Palletering. Denne
% modellen blir hentet inn (run) i flere andre relaterte programmer. 

%% Robotarm for Palletering

clc;
clear;

syms q1 q2 q3 q4 q5 q6 real
syms L1 L2 L3 L4 L5 L6 real


%% 6 DOF Robotarm

% Lenkelengder
robot.L1 = L1;
robot.L2 = L2;
robot.L3 = L3;
robot.L4 = L4;
robot.L5 = L5;
robot.L6 = L6; 

% Homogen transformasjonsmatrise (transformasjon av robot)
DHmatrix = @(p) [ ...
    cos(p.theta), -sin(p.theta)*cos(p.alpha),  sin(p.theta)*sin(p.alpha), p.a*cos(p.theta);
    sin(p.theta),  cos(p.theta)*cos(p.alpha), -cos(p.theta)*sin(p.alpha), p.a*sin(p.theta);
    0,           sin(p.alpha),             cos(p.alpha),            p.d;
    0,           0,                       0,                    1];


% Robot modell
robot.DH(1) = struct('type','R', 'a',0, 'alpha',(sym(pi)/2), 'd',L1, 'theta',q1);
robot.DH(2) = struct('type','R', 'a',L2, 'alpha',0, 'd',0, 'theta',q2+pi/2);
robot.DH(3) = struct('type','R', 'a',L3, 'alpha',0, 'd',0, 'theta',q3);
robot.DH(4) = struct('type','R', 'a',0, 'alpha',(-sym(pi)/2), 'd',L4, 'theta',q4-(sym(pi)/2));
robot.DH(5) = struct('type','R', 'a',0, 'alpha',(sym(pi)/2), 'd',L5, 'theta',q5);
robot.DH(6) = struct('type','R', 'a',0, 'alpha',0, 'd',L6, 'theta',q6);

% Transformasjon
T01 = DHmatrix(robot.DH(1));
T12 = DHmatrix(robot.DH(2));
T23 = DHmatrix(robot.DH(3));
T34 = DHmatrix(robot.DH(4));
T45 = DHmatrix(robot.DH(5));
T56 = DHmatrix(robot.DH(6));

T06 = T01*T12*T23*T34*T45*T56;

% Sizing the robot
robot.L1 = 0.1;
robot.L2 = 0.9;
robot.L3  = 0.8;
robot.L4  = 0.2;
robot.L5  = 0.15;
robot.L6  = 0.15;

% Opprett SerialLink-modell

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


% Lag en robot
Palletizing = SerialLink(L, 'name', 'Palletizing Robot');

q = deg2rad([0 0 0 0 0 0]);
T = Palletizing.fkine(q);


% Implementer leddbegrensninger for roboter
Palletizing.qlim = [ ...
    deg2rad([70  230]);         % Ledd 1: revolutt
    -pi/2         pi/2;         % Ledd 2: revolutt
    deg2rad([-180 180]);        % Ledd 3: revolutt
    -pi         pi;             % Ledd 4: revolutt
    -pi         pi;             % Ledd 5: revolutt
    -pi         pi];            % Ledd 6: revolutt

%% PowerPick 10 vakuumsuger

gripper.name = 'PowerPick 10';

gripper.tcp = [0 0 0.1];        % [m]

%   TCP-transformasjon i forhold til robotbasen
T6TCP = transl(gripper.tcp);
T0TCP = T06 * T6TCP;

Palletizing.tool = transl(gripper.tcp);     % Kobler endeeffektoren til TCP
