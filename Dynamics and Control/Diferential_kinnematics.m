close all;
clear all;
clc;

%% robot model construction

%------------------------------------------------------
% 1. Define the physical robot and link dynamics
%------------------------------------------------------
% Link dimensions and masses (SI units)

L1 = 0.1;   m1 = 1.0;
L2 = 0.9;   m2 = 1.0;
L3 = 0.8;   m3 = 0.9;
L4 = 0.2;   m4 = 0.4;
L5 = 0.15;  m5 = 0.3;
L6 = 0.15;  m6 = 0.3;

% Link Center of mass (COM) distances (m)
d1 = L1/2;  d2 = L2/2;  d3 = L3/2;
d4 = L4/2;  d5 = L5/2;  d6 = L6/2;

% Gravitational acceleration (m/s^2)
g0 = 9.81;      

% Simplified rectangular, uniform links, cross section w x h (m).
w = 0.1;   h = 0.05;

% COM positions in each link frame
% COM positions in local DH frames
r1 = [0 -d1 0];
r2 = [-d2 0 0];
r3 = [-d3 0 0];
r4 = [0 d4 0];
r5 = [0 -d5 0];
r6 = [0 0 -d6];

% Inertia tensors about each link's COM
% Link 1 - length along local Y-axis
I1 = diag([m1*(L1^2+h^2)/12, ...
           m1*(w^2+h^2)/12, ...
           m1*(L1^2+w^2)/12]);

% Link 2 - length along local X-axis
I2 = diag([m2*(w^2+h^2)/12, ...
           m2*(L2^2+h^2)/12, ...
           m2*(L2^2+w^2)/12]);

% Link 3 - length along local X-axis
I3 = diag([m3*(w^2+h^2)/12, ...
           m3*(L3^2+h^2)/12, ...
           m3*(L3^2+w^2)/12]);

% Link 4 - length along local Y-axis
I4 = diag([m4*(L4^2+h^2)/12, ...
           m4*(w^2+h^2)/12, ...
           m4*(L4^2+w^2)/12]);

% Link 5 - length along local Y-axis
I5 = diag([m5*(L5^2+h^2)/12, ...
           m5*(w^2+h^2)/12, ...
           m5*(L5^2+w^2)/12]);

% Link 6 - length along local Z-axis
I6 = diag([m6*(L6^2+h^2)/12, ...
           m6*(L6^2+w^2)/12, ...
           m6*(w^2+h^2)/12]);

%-------------------------------------------------------
% 2. Define actuator and friction parameters
%------------------------------------------------------
% These values are illustrative. Real values should be based on the chosen motors and transmissions.
% Motor and friction parameters (illustrative)
Jm = 1e-4;       % Motor rotor inertia (kg m^2)
B  = 0.01;       % Viscous friction
Tc = [0 0];      % Coulomb friction [positive, negative]
G  = 10;          % Gear ratio

% Define links: standard DH, planar 2R robot
% DH parameters are unchanged from the group's 6R model.
links1 = Revolute('d', L1, 'a', 0,  'alpha',  pi/2, 'offset', 0, ...
                    'm', m1, 'r', r1, 'I', I1, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links2 = Revolute('d', 0,  'a', L2, 'alpha',  0,    'offset', pi/2, ...
                    'm', m2, 'r', r2, 'I', I2, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links3 = Revolute('d', 0,  'a', L3, 'alpha',  0,    'offset', 0, ...
                    'm', m3, 'r', r3, 'I', I3, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links4 = Revolute('d', L4, 'a', 0,  'alpha', -pi/2, 'offset', -pi/2, ...
                    'm', m4, 'r', r4, 'I', I4, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links5 = Revolute('d', L5, 'a', 0,  'alpha',  pi/2, 'offset', 0, ...
                    'm', m5, 'r', r5, 'I', I5, ... 
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links6 = Revolute('d', L6, 'a', 0,  'alpha',  0,    'offset', 0, ...
                    'm', m6, 'r', r6, 'I', I6, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);


% Assemble the robot
robot = SerialLink([links1 links2 links3 links4 links5 links6] , 'name', 'Palletizing robot');

% Gravity points in the negative y direction
robot.gravity = [0 9.81 0];

% Display robot
robot.teach([0 0 0 0 0 0]);