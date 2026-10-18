%% 6DOF Robot Arm for Palletizing

clc;
clear;

syms q1 q2 q3 q4 q5 q6 real
syms L1 L2 L3 L4 L5 L6 real


import ETS3.*

%% 6 DOF Robotarm

% Lenkelengder
robot.L1 = L1;
robot.L2 = L2;
robot.L3 = L3;
robot.L4 = L4;
robot.L5 = L5;
robot.L6 = L6; 

% Homogenous transformation matrix (transformation of robot)
DHmatrix = @(p) [ ...
    cos(p.theta), -sin(p.theta)*cos(p.alpha),  sin(p.theta)*sin(p.alpha), p.a*cos(p.theta);
    sin(p.theta),  cos(p.theta)*cos(p.alpha), -cos(p.theta)*sin(p.alpha), p.a*sin(p.theta);
    0,           sin(p.alpha),             cos(p.alpha),            p.d;
    0,           0,                       0,                    1];


% Robot model
robot.DH(1) = struct('type','R', 'a',0, 'alpha',pi/2, 'd',L1, 'theta',q1);
robot.DH(2) = struct('type','R', 'a',L2, 'alpha',0, 'd',0, 'theta',q2+pi/2);
robot.DH(3) = struct('type','R', 'a',L3, 'alpha',0, 'd',0, 'theta',q3);
robot.DH(4) = struct('type','R', 'a',0, 'alpha',-pi/2, 'd',L4, 'theta',q4-pi/2);
robot.DH(5) = struct('type','R', 'a',0, 'alpha',pi/2, 'd',L5, 'theta',q5);
robot.DH(6) = struct('type','R', 'a',0, 'alpha',0, 'd',L6, 'theta',q6);

% Transformation (Palletizing Robot)
T01 = DHmatrix(robot.DH(1));
T12 = DHmatrix(robot.DH(2));
T23 = DHmatrix(robot.DH(3));
T34 = DHmatrix(robot.DH(4));
T45 = DHmatrix(robot.DH(5));
T56 = DHmatrix(robot.DH(6));

T06 = T01*T12*T23*T34*T45*T56

% Sizing the robot (Legg inn faktiske lenkelengder)
robot.L1 = 0.4;
robot.L2 = 0.5;
robot.L3  = 0.4;
robot.L4  = 0.3;
robot.L5  = 0.35;
robot.L6  = 0.3;

% Create SerialLink model

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


% Create robot
Palletizing = SerialLink(L, 'name', 'Palletizing Robot');

q = deg2rad([0 0 0 0 0 0]);
T = Palletizing.fkine(q)


% Implement Robots Joint limits
Palletizing.qlim = [ ...
    -pi         pi;          % Joint 1: revolute
    -pi         pi;          % Joint 2: revolute
    deg2rad([-160 160]);     % Joint 3: revolute
    -pi         pi;          % Joint 4: revolute
    -pi         pi;          % Joint 5: revolute
    -pi         pi];         % Joint 6: revolute

%% PowerPick 10 Vacuum Gripper

gripper.name = 'PowerPick 10';

gripper.tcp = [0 0 0.1];            % [m]

% TCP transformation relative to robotbase
T6TCP = transl(gripper.tcp);
T0TCP = T06 * T6TCP;

Palletizing.tool = transl(gripper.tcp);     % Attaches TCP to robot


%% Workspace 

%% Workspace transformations - All relative to the robotbase {0} (world coordinate system) 

% Transformation from robot {0} relative to conveyorbelt {C}
conveyorPosition = [0, -1.25, 0];
conveyorRotation = 0;


T0C = transl(conveyorPosition) * trotz(conveyorRotation);


% Transformation from robot {0} relative to pallet {P}
palletPosition = [1.0, 0.2, 0];
palletRotation = pi/2;


T0P = transl(palletPosition) * trotz(palletRotation);


% Transformation from camera {K} relative to conveyor {C}
cameraPosition = [0, 0, 2.0];
cameraRotation = pi;            % Points down at conveyor

TCK = transl(cameraPosition)*trotx(cameraRotation);

T0K = T0C * TCK;                % Transformation from camera {K} relative to robotbase {0} (since it is attached above) 

% Transformation from box {B} relative to conveyor {C} - For simulation
% only - later we use the camera to detect position on conveyor
boxPosition = [0, 0.25, 0.4];
boxRotation = 0;

TCB = transl(boxPosition)*trotz(boxRotation);

T0B = T0C * TCB;                % Transformation from box {B} relative to robotbase {0}

% Transformation from camera {K} relative to box {B} (For camera detecting box - No actual data yet) 
x_detected = 0.10;              % Detected x-position
y_detected = 0.05;              % Detected y-position
z_detected = 0.40;              % Detected z-position

TKB = transl(x_detected, y_detected, z_detected);

T0B_K = T0K *  TKB;             % Transformation of detected box position from camera {K} relative to robotbase {0}


% Workspace Dimensions
conveyor.length     = 0.6;      % Has to fit a box (400x400x400mm) -> 10 cm space on each side of box
conveyor.width      = 1.0;      % Not important
conveyor.height    = 0.4;      % Max. vertical reach 1m (0.4m + 0.4m (box) = 0.8) -> 0.2m space to lift box up 

pallet.length       = 1.20;     % Standard European Pallet
pallet.width        = 0.80;
pallet.height       = 0.144;

box.size            = 0.4;      % Box (400x400x400mm) -> 0.4m 


camera.length       = 0.12;
camera.width        = 0.08;
camera.height       = 0.06;

camera.z            = 2.00;     % Height above ground


%%-----------------------------------------------------------------------------------------------------------------------%%


%% Vizualising Robot

q0 = deg2rad([-75 -20 -20 -30 90 0]);

figure('Name','6DOF Palletizing Workspace');

% Robot
Palletizing.plot(q0, ...
    'workspace', [-1.8 1.8 -1.8 1.8 -0.1 1.8]);

% Calculate TCP transformation
T0TCP_calculated = Palletizing.fkine(q0);

disp('TCP position [m]:');               % To control the location of the TCP when we move the robot
disp(transl(T0TCP_calculated));


%% Workspace visualization using patch() 
% References: 
% https://se.mathworks.com/help/matlab/visualize/multifaceted-patches.html 
% + https://se.mathworks.com/help/matlab/ref/matlab.graphics.primitive.patch-properties.html 
% + ChatGPT

hold on;

% Faces for rectangular 3D object
F = [ ...
    1 2 3 4;   % Bottom
    5 6 7 8;   % Top
    1 2 6 5;   % Front
    2 3 7 6;   % Right
    3 4 8 7;   % Back
    4 1 5 8];  % Left

% Conveyor

l = conveyor.length;
w = conveyor.width;
h = conveyor.height;

% Vertices for conveyor
V = [ ...
    -l/2, -w/2, 0;
     l/2, -w/2, 0;
     l/2,  w/2, 0;
    -l/2,  w/2, 0;
    -l/2, -w/2, h;
     l/2, -w/2, h;
     l/2,  w/2, h;
    -l/2,  w/2, h];

% Convert to homogeneous coordinates
V_hc = [V, ones(size(V,1),1)]';

% Transform from {C} to {0}     - Implement correct transformation
V_world_hc = T0C * V_hc;

% Convert back to Cartesian coordinates
V_world = V_world_hc(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0.4 0.7 0.9], ...
      'FaceAlpha', 0.7, ...
      'EdgeColor', 'k');

% Pallet

l = pallet.length;
w = pallet.width;
h = pallet.height;

V = [
    -l/2 -w/2 0;
     l/2 -w/2 0;
     l/2  w/2 0;
    -l/2  w/2 0;
    -l/2 -w/2 h;
     l/2 -w/2 h;
     l/2  w/2 h;
    -l/2  w/2 h
];


% Convert to homogeneous coordinates
V_hp = [V, ones(size(V,1),1)]';

% Transform from {P} to {0}     - Implement correct transformation
V_world_hp = T0P * V_hp;

% Convert back to Cartesian coordinates
V_world = V_world_hp(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0.6 0.4 0.2], ...
      'FaceAlpha', 0.8, ...
      'EdgeColor', 'k');


% Box on conveyor

l = box.size;
w = box.size;
h = box.size;

V = [
    -l/2 -w/2 0;
     l/2 -w/2 0;
     l/2  w/2 0;
    -l/2  w/2 0;
    -l/2 -w/2 h;
     l/2 -w/2 h;
     l/2  w/2 h;
    -l/2  w/2 h
];

% Convert to homogeneous coordinates
V_hb = [V, ones(size(V,1),1)]';

% Transform from {B} to {0}     - Implement correct transformation
V_world_hb = T0B * V_hb;

% Convert back to Cartesian coordinates
V_world = V_world_hb(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0.8 0.5 0.2], ...
      'FaceAlpha', 0.9, ...
      'Edgecolor', 'k');


% Camera

l = camera.length;
w = camera.width;
h = camera.height;

V = [
    -l/2 -w/2 0;
     l/2 -w/2 0;
     l/2  w/2 0;
    -l/2  w/2 0;
    -l/2 -w/2 h;
     l/2 -w/2 h;
     l/2  w/2 h;
    -l/2  w/2 h
];

% Convert to homogeneous coordinates
V_hk = [V, ones(size(V,1),1)]';

% Transform from {K} to {0}     - Implement correct transformation
V_world_hk = T0K * V_hk;

% Convert back to Cartesian coordinates
V_world = V_world_hk(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0 0 0.15], ...
      'FaceAlpha', 1.0, ...
      'Edgecolor', 'k');


%% Plotting

% Coordinate frames
trplot(T0C, 'frame', 'C', 'length', 0.2);       % Conveyor
trplot(T0P, 'frame', 'P', 'length', 0.2);       % Pallet
trplot(T0B, 'frame', 'B', 'length', 0.2);       % Box (has to change when we implement camera)
trplot(T0K, 'frame', 'K', 'length', 0.2);       % Camera

% Plot settings
axis equal;

xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
title('6DOF Palletizing Robot Workspace')
view(3);
grid on;

% Draw robot

figure
Palletizing.plot(q0)
Palletizing.teach(q0)