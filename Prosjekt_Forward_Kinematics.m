% Toolbox 

clc;
clear;

import ETS3.*

%% Lenkelengder - usikker på lengdene
L1 = 1;
L2 = 3;
L3 = 3;
L4 = 1.5;
L5 = 1.5;
L6 = 1.5; 

%% Link 1
% theta1, d=L1, a=0, alpha=pi/2

E1 = Rz('q1') * Tz(L1) * Rx(pi/2);

%% Link 2
% theta2 + pi/2, d=0, a=L2, alpha=0

E2 = Rz('q2') * Rz(pi/2) * Tx(L2);

%% Link 3
% theta3, d=0, a=L3, alpha=0

E3 = Rz('q3') * Tx(L3);

%% Link 4
% theta4 - pi/2, d=L4, a=0, alpha=-pi/2

E4 = Rz('q4') * Rz(-pi/2) * Tz(L4) * Rx(-pi/2);

%% Link 5
% theta5, d=L5, a=0, alpha=pi/2
E5 = Rz('q5') * Tz(L5) * Rx(pi/2);

%% Link 6
% theta6, d=L6, a=0, alpha=0
E6 = Rz('q6') * Tz(L6);


%% Hele 6-DOF roboten
E = E1 * E2 * E3 * E4 * E5 * E6;

E.fkine([0 0 0 0 0 0], 'deg')
E.teach
