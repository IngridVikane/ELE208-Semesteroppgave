%% Forward kinematics - første 4 ledd
clc;
clear;

% Symbolske variabler
syms theta1 theta2 theta3 theta4 real
syms L1 L2 L3 L4 real

%% Transformasjon 0T1
T01 = [
    cos(theta1), -0,            sin(theta1), 0;
    sin(theta1),  0,           -cos(theta1), 0;
    0,            1,            0,           L1;
    0,            0,            0,           1
];

%% Transformasjon 1T2
T12 = [
    cos(theta2 + pi/2), -sin(theta2 + pi/2), 0, L2*cos(theta2 + pi/2);
    sin(theta2 + pi/2),  cos(theta2 + pi/2), 0, L2*sin(theta2 + pi/2);
    0,                   0,                  1, 0;
    0,                   0,                  0, 1
];

%% Transformasjon 2T3
T23 = [
    cos(theta3), -sin(theta3), 0, L3*cos(theta3);
    sin(theta3),  cos(theta3), 0, L3*sin(theta3);
    0,            0,           1, 0;
    0,            0,           0, 1
];

%% Transformasjon 3T4
T34 = [
    cos(theta4 - pi/2), 0, -sin(theta4 - pi/2), 0;
    sin(theta4 - pi/2), 0,  cos(theta4 - pi/2), 0;
    0,                  -1,  0,                  L4;
    0,                   0,  0,                  1
];

%% Forward kinematics - matrisemultiplikasjonen
T04 = T01 * T12 * T23 * T34;

% Forenkle det symbolske resultatet
T04_hand = simplify(T04, 'Steps', 10)

%########################################################################%

%% Sammenligne T04 for hånd og vha. toolbox

%% Symbolske variabler
syms theta1 theta2 theta3 theta4 real
syms L1 L2 L3 L4 real

%% Peter Corke Robotics Toolbox

L(1) = Link('revolute',  'd', L1, 'a', 0, 'alpha', pi/2);

L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0, 'offset', pi/2);

L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);

L(4) = Link('revolute', 'd', L4, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2);

robot = SerialLink(L, 'name', 'RRRR Robot');

%% Symbolsk FK fra Corke
T04_toolbox = robot.fkine([theta1 theta2 theta3 theta4]); 

%% Opprydding
T04_clean = simplify(vpa(T04_toolbox, 2));
T04_clean

% Alle små tall = 0

