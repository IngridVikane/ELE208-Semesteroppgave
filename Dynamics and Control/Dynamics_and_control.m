close all; clear; clc;

run('InverseKinematics_Palletizing12boxes.m');

%% 1. Define the physical robot and link dynamics

% Link dimensions and masses
m = [0.6750 6.0750 5.4000 0.2160 0.1620 0.1620];

Palletizing.links(1).m = m(1);
Palletizing.links(2).m = m(2);
Palletizing.links(3).m = m(3);
Palletizing.links(4).m = m(4);
Palletizing.links(5).m = m(5);
Palletizing.links(6).m = m(6);

L = [0.10 0.90 0.80 0.20 0.15 0.15];
w = [0.05 0.05 0.05 0.02 0.02 0.02];
h = [0.05 0.05 0.05 0.02 0.02 0.02];
g0 = 9.81;      % Gravitational acceleration (m/s^2)

Palletizing.gravity = [0 0 g0];

% COM positions in each link frame
r1 = [0 -L(1)/2 0]; 
r2 = [-L(2)/2 0 0];
r3 = [-L(3)/2 0 0];
r4 = [0 L(4)/2 0];
r5 = [0 -L(5)/2 0];
r6 = [0 0 -L(6)/2];


Palletizing.links(1).r = r1;
Palletizing.links(2).r = r2;
Palletizing.links(3).r = r3;
Palletizing.links(4).r = r4;
Palletizing.links(5).r = r5;
Palletizing.links(6).r = r6;


% Inertia tensors about each link's COM
I1 = diag([m(1)*(L(1)^2+h(1)^2)/12, ...
    m(1)*(w(1)^2+h(1)^2)/12, ...
    m(1)*(L(1)^2+w(1)^2)/12]);

I2 = diag([m(2)*(w(2)^2+h(2)^2)/12, ...
    m(2)*(L(2)^2+h(2)^2)/12, ...
    m(2)*(L(2)^2+w(2)^2)/12]);

I3 = diag([m(3)*(w(3)^2+h(3)^2)/12, ...
    m(3)*(L(3)^2+h(3)^2)/12, ...
    m(3)*(L(3)^2+w(3)^2)/12]);

I4 = diag([m(4)*(L(4)^2+h(4)^2)/12, ...
    m(4)*(w(4)^2+h(4)^2)/12, ...
    m(4)*(L(4)^2+w(4)^2)/12]);

I5 = diag([m(5)*(L(5)^2+h(5)^2)/12, ...
    m(5)*(w(5)^2+h(5)^2)/12, ...
    m(5)*(L(5)^2+w(5)^2)/12]);

I6 = diag([m(6)*(L(6)^2+w(6)^2)/12, ...
    m(6)*(L(6)^2+h(6)^2)/12, ...
    m(6)*(w(6)^2+h(6)^2)/12]);

Palletizing.links(1).I = I1;
Palletizing.links(2).I = I2;
Palletizing.links(3).I = I3;
Palletizing.links(4).I = I4;
Palletizing.links(5).I = I5;
Palletizing.links(6).I = I6;

%% 2. Define actuator and friction parameters

Jm = 1e-4;       % Motor rotor inertia (kg m^2)
B  = 0.01;       % Viscous friction
Tc = [0 0];      % Coulomb friction [positive, negative]
G  = 10;          % Gear ratio

for i = 1:6
    Palletizing.links(i).Jm = Jm;
    Palletizing.links(i).B  = B;
    Palletizing.links(i).Tc = Tc;
    Palletizing.links(i).G  = G;
end

Palletizing.dyn()