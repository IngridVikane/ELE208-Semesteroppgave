close all; clear; clc;

run('InverseKinematics_Palletizing12boxes.m');

%% Center of Mass (COM)

% Link dimensions and masses
Palletizing.links(1).m = 0.675;
Palletizing.links(2).m = 6.075;
Palletizing.links(3).m = 5.400;
Palletizing.links(4).m = 0.2160;
Palletizing.links(5).m = 0.1620;
Palletizing.links(6).m = 0.1620;

m = [0.6750 6.0750 5.4000 0.2160 0.1620 0.1620];
L = [0.10 0.90 0.80 0.20 0.15 0.15];
w = [0.05 0.05 0.05 0.02 0.02 0.02];
h = [0.05 0.05 0.05 0.02 0.02 0.02];
g0 = 9.81;      % Gravitational acceleration (m/s^2)


% COM positions in each link frame
                    % dh-table: 
r1 = [0 -0.050 0];  % d-parameter   (z-axis)
r2 = [-0.450 0 0];  % a-parameter   (x-axis)
r3 = [-0.400 0 0];  % a-parameter
r4 = [0 0 -0.100];  % d-parameter
r5 = [0 0 -0.075];  % d-parameter
r6 = [0 0 -0.075];  % d-parameter


% Inertia tensors about each link's COM
I1 = diag([m(1)*(w(1)^2+h(1)^2)/12, ...
    m(1)*(L(1)^2+h(1)^2)/12, ...
    m(1)*(L(1)^2+w(1)^2)/12]);

I2 = diag([m(2)*(w(2)^2+h(2)^2)/12, ...
    m(2)*(L(2)^2+h(2)^2)/12, ...
    m(2)*(L(2)^2+w(2)^2)/12]);

I3 = diag([m(3)*(w(3)^2+h(3)^2)/12, ...
    m(3)*(L(3)^2+h(3)^2)/12, ...
    m(3)*(L(3)^2+w(3)^2)/12]);

I4 = diag([m(4)*(w(4)^2+h(4)^2)/12, ...
    m(4)*(L(4)^2+h(4)^2)/12, ...
    m(4)*(L(4)^2+w(4)^2)/12]);

I5 = diag([m(5)*(w(5)^2+h(5)^2)/12, ...
    m(5)*(L(5)^2+h(5)^2)/12, ...
    m(5)*(L(5)^2+w(5)^2)/12]);

I6 = diag([m(6)*(w(6)^2+h(6)^2)/12, ...
    m(6)*(L(6)^2+h(6)^2)/12, ...
    m(6)*(L(6)^2+w(6)^2)/12]);








