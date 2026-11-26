%% Dynamisk modell for aktuatorvalg.
% % 
%    Copyright:     HVL
%    Project:	    ELE208 - Robotmanipulatorer
%    Author:        Sigurd Hodne  
%    Coauthor:      Ingrid Vikane: la til faktiske masser, fikk nye
%                   verdier og figurer
%    Date created:  Høst 2026
%    Input: 'Palletizing_Robotarm' og 'pallet_cycles'
%
%    Output: Leddmomenter med og uten eske, treghetsmatrise, tyngdekraft-,
%            Coriolis- og friksjonsbidrag
%---------------------------------------------------------------------%
%%
% Note: 
% Based on PlanarDynamics - V2.m from lectures.
% and Dynamics_ans_control.m
%
% Dette programmet bygger en dynamisk modell av roboten og beregner nødvend
% ige leddmomenter langs palleteringsbanen med og uten eske. Momentene blir
% delt opp i treghets-, Coriolis-, tyngdekraft- og friksjonsbidrag.
%
close all; clear; clc;

%% robot model construction

run('Palletizing_Robotarm.m');
load('pallet_cycles.mat');

%% Masser - Solide/massive aluminiumslenker
% Lenkelengder [m]
L = [0.10 0.90 0.80 0.20 0.15 0.15];

% Bredde [m]
w = [0.05 0.05 0.05 0.02 0.02 0.02];

h = [0.05 0.05 0.05 0.02 0.02 0.02];

% Aluminiums tetthet [kg/m^3]
p = 2700;

% Volum [m^3]
V = L.*h.*w;

% Lenke masser [kg]
m = p .* V;

for i = 1:6
    fprintf('Link %d: %.3f kg\n', i, m(i));
end

% Total masse
m_total = sum(m);

disp(m)
fprintf('Total masse er: ')
disp(m_total)

%% Select palletizing cycle
cycle = 1;

t   = cycles(cycle).t;
Q   = cycles(cycle).q;
QD  = cycles(cycle).qd;
QDD = cycles(cycle).qdd;

N = size(Q,1);

%% Verify loaded trajectory

size(t)
size(Q)
size(QD)
size(QDD)

assert(numel(t) == N);
assert(size(Q,2) == 6);
assert(isequal(size(Q),size(QD),size(QDD)));

%------------------------------------------------------
% 1. Define the physical robot and link dynamics
%------------------------------------------------------
% Link dimensions and masses (SI units)
L1 = L(1);  m1 = m(1);
L2 = L(2);  m2 = m(2);
L3 = L(3);  m3 = m(3);
L4 = L(4);  m4 = m(4);
L5 = L(5);  m5 = m(5);
L6 = L(6);  m6 = m(6);

% Link Center of mass (COM) distances (m)
d1 = L1/2;  d2 = L2/2;  d3 = L3/2;
d4 = L4/2;  d5 = L5/2;  d6 = L6/2;

% Gravitational acceleration (m/s^2)
g0 = 9.81;      

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
I1 = diag([m(1)*(L(1)^2+h(1)^2)/12, ...
           m(1)*(w(1)^2+h(1)^2)/12, ...
           m(1)*(L(1)^2+w(1)^2)/12]);

% Link 2 - length along local X-axis
I2 = diag([m(2)*(w(2)^2+h(2)^2)/12, ...
           m(2)*(L(2)^2+h(2)^2)/12, ...
           m(2)*(L(2)^2+w(2)^2)/12]);

% Link 3 - length along local X-axis
I3 = diag([m(3)*(w(3)^2+h(3)^2)/12, ...
           m(3)*(L(3)^2+h(3)^2)/12, ...
           m(3)*(L(3)^2+w(3)^2)/12]);

% Link 4 - length along local Y-axis
I4 = diag([m(4)*(L(4)^2+h(4)^2)/12, ...
           m(4)*(w(4)^2+h(4)^2)/12, ...
           m(4)*(L(4)^2+w(4)^2)/12]);

% Link 5 - length along local Y-axis
I5 = diag([m(5)*(L(5)^2+h(5)^2)/12, ...
           m(5)*(w(5)^2+h(5)^2)/12, ...
           m(5)*(L(5)^2+w(5)^2)/12]);

% Link 6 - length along local Z-axis
I6 = diag([m(6)*(L(6)^2+h(6)^2)/12, ...
           m(6)*(L(6)^2+w(6)^2)/12, ...
           m(6)*(w(6)^2+h(6)^2)/12]);

%-------------------------------------------------------
% 2. Define actuator and friction parameters
%------------------------------------------------------
% These values are illustrative. Real values should be based on the chosen motors and transmissions.
% Motor and friction parameters (illustrative)
act.mass  = [9.5 9.5 9.5 1.639 1.639 1.639];   % Vekt [kg]: eRob170I (ledd 1–3), eRob90I (ledd 4–6)
act.speed = [40 40 40 60 60 60];               % Maks utgangshastighet [o/min]

Jm = 1e-4;       % Motor rotor inertia (kg m^2)
B  = 0.01;       % Viscous friction
Tc = [0 0];      % Coulomb friction [positive, negative]
G  = 10;         % Gear ratio

% Define links: standard DH.
links1 = Revolute('d', L(1), 'a', 0,  'alpha',  pi/2, 'offset', 0, ...
                    'm', m(1), 'r', r1, 'I', I1, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links2 = Revolute('d', 0,  'a', L(2), 'alpha',  0,    'offset', pi/2, ...
                    'm', m(2), 'r', r2, 'I', I2, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links3 = Revolute('d', 0,  'a', L(3), 'alpha',  0,    'offset', 0, ...
                    'm', m(3), 'r', r3, 'I', I3, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links4 = Revolute('d', L(4), 'a', 0,  'alpha', -pi/2, 'offset', -pi/2, ...
                    'm', m(4), 'r', r4, 'I', I4, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links5 = Revolute('d', L(5), 'a', 0,  'alpha',  pi/2, 'offset', 0, ...
                    'm', m(5), 'r', r5, 'I', I5, ... 
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);
links6 = Revolute('d', L(6), 'a', 0,  'alpha',  0,    'offset', 0, ...
                    'm', m(6), 'r', r6, 'I', I6, ...
                    'Jm', Jm, 'B', B, 'Tc', Tc, 'G', G);




% Assemble the robot
robot = SerialLink([links1 links2 links3 links4 links5 links6] , 'name', 'Palletizing robot');

% Gravity points in the negative z direction
robot.gravity = [0 0 g0];

% Display robot movement
figure;
robot.plot(Q);

%% End-effector trajectory

% Allocate position matrix
P = zeros(N,3);

for k = 1:N

    % Forward kinematics
    T_actual = robot.fkine(Q(k,:));

    % Extract end-effector position
    P(k,:) = transl(T_actual);

end

% Plot end-effector trajectory
figure;

plot3(P(:,1),P(:,2),P(:,3),'LineWidth',2);

grid on;
axis equal;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-effector trajectory');

%% Griper (2 kg) – roboten kjører alltid med endeeffektoren montert
% Massesenter antatt 0,05 m ut fra flensen (halvveis til TCP (0.1m))
m_gripper = 2.0;
robot.payload(m_gripper, [0 0 0.05]);

% %% Recursive Newton Euler method
% Inverse dynamics: required joint torques
% Joint configuration, velocity and acceleration

tau = robot.rne(Q, QD, QDD);

%% Inertia Matrix

%-----------------------------------------------------
% Inertia matrix
%-----------------------------------------------------
nQ = size(Q, 1);
M_values = zeros(6,6,nQ);
M_diag = zeros(nQ,6);

for k = 1:nQ

    % Joint configuration at time step k
    q = Q(k,:);

    % Calculate inertia matrix
    M = robot.inertia(q);

    % Store complete inertia matrix
    M_values(:,:,k) = M;

    % Store diagonal elements
    M_diag(k,:) = diag(M)';

end

%% Joint motion and dynamics

figure;
tiledlayout(4,1);

nexttile;
plot(t, rad2deg(Q));
title('Joint positions');
ylabel('Angle (deg)');
grid on;

nexttile;
plot(t, QD);
title('Joint velocities');
ylabel('Velocity (rad/s)');
grid on;

nexttile;
plot(t, QDD);
title('Joint accelerations');
ylabel('Acceleration (rad/s^2)');
grid on;

nexttile;
plot(t, tau);
title('Joint torques');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

legend('Joint 1','Joint 2','Joint 3', ...
    'Joint 4','Joint 5','Joint 6');

figure;
tiledlayout(6,6, ...
    'TileSpacing','compact', ...
    'Padding','compact');

for i = 1:6
    for j = 1:6

        Mij = squeeze(M_values(i,j,:));

        nexttile;
        plot(t, Mij);

        title(sprintf('M_{%d%d}',i,j));
        grid on;

    end
end


%% Gravity load
% Gravity-compensation torque
tau_g = robot.gravload(Q);

% Plot gravity load
figure;

plot(t, tau_g, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Gravity Compensation Torque');

legend('Joint 1','Joint 2','Joint 3', ...
    'Joint 4','Joint 5','Joint 6');

%% Compare total torque and gravity torque

figure;
tiledlayout(3,2);

for j = 1:6

    nexttile;

    plot(t, tau(:,j), 'LineWidth', 1.5);
    hold on;

    plot(t, tau_g(:,j), '--', 'LineWidth', 1.5);

    grid on;
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title(sprintf('Joint %d',j));

    legend('Total torque','Gravity torque');

end

sgtitle('Total Torque vs Gravity Compensation');


%% Coriolis term

% Allocate matrices
C_values = zeros(6,6,nQ);
tau_c = zeros(nQ,6);

for k = 1:nQ

    % Joint configuration and velocity
    q = Q(k,:);
    qd = QD(k,:);

    % Calculate Coriolis matrix
    C = robot.coriolis(q,qd);

    % Store complete Coriolis matrix
    C_values(:,:,k) = C;

    % Calculate Coriolis and centrifugal torque
    tau_c(k,:) = (C * qd')';

end

% Plot Coriolis and centrifugal torque
figure;

plot(t,tau_c,'LineWidth',1.5);

grid on;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Coriolis and Centrifugal Torque');

legend('Joint 1','Joint 2','Joint 3', ...
       'Joint 4','Joint 5','Joint 6');

% Plot Coriolis matrix elements
figure;

tiledlayout(6,6, ...
    'TileSpacing','compact', ...
    'Padding','compact');

for i = 1:6
    for j = 1:6

        Cij = squeeze(C_values(i,j,:));

        nexttile;
        plot(t,Cij);

        title(sprintf('C_{%d%d}',i,j));
        grid on;

    end
end

sgtitle('Coriolis Matrix');

%% Friction term

% Allocate matrix
tau_f = zeros(nQ,6);

for k = 1:nQ

    % Joint velocity at time step k
    qd = QD(k,:);

    % Calculate friction compensation torque
    tau_f(k,:) = -robot.friction(qd);

end
% Plot friction torque
figure;

plot(t,tau_f,'LineWidth',1.5);

grid on;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Friction Compensation Torque');

legend('Joint 1','Joint 2','Joint 3', ...
    'Joint 4','Joint 5','Joint 6');

%% Inertia torque

% Allocate matrix
tau_m = zeros(nQ,6);

for k = 1:nQ

    % Inertia matrix at time step k
    M = M_values(:,:,k);

    % Joint acceleration at time step k
    qdd = QDD(k,:);

    % Calculate inertia torque
    tau_m(k,:) = (M * qdd')';

end

% Plot inertia torque
figure;

plot(t,tau_m,'LineWidth',1.5);

grid on;
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Inertia Torque');

legend('Joint 1','Joint 2','Joint 3', ...
    'Joint 4','Joint 5','Joint 6');

%% Verification of dynamic equation

% Sum of all torque components
tau_calculated = tau_m + tau_c + tau_g + tau_f;

% Difference from Recursive Newton Euler
tau_error = tau - tau_calculated;

% Maximum absolute error for each joint
max_error = max(abs(tau_error))

%% Compare all torque components

figure;
tiledlayout(3,2);

for j = 1:6

    nexttile;

    plot(t,tau(:,j),'k','LineWidth',2);
    hold on;

    plot(t,tau_m(:,j),'LineWidth',1.5);
    plot(t,tau_c(:,j),'LineWidth',1.5);
    plot(t,tau_g(:,j),'LineWidth',1.5);
    plot(t,tau_f(:,j),'LineWidth',1.5);

    grid on;
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title(sprintf('Joint %d',j));

    legend('Total','Inertia','Coriolis', ...
        'Gravity','Friction');

end

sgtitle('Dynamic Torque Components');

%{

Important: In the classic MATLAB Robotics Toolbox, the payload is incorporated into the terminal link's 
dynamic model by setting its mass and COM properties, rather than simply adding its mass to the existing 
terminal link. It does not automatically calculate the combined COM and inertia tensor of the original link plus payload.
% fact check:
% Inspect link 2 before adding payload
robot.links(2).m
robot.links(2).r

% Add payload
robot.payload(1, [0 0 0]);

% Inspect link 2 afterwards
robot.links(2).m
robot.links(2).r

%}

%% Effect of payload

% Inspect link 6 before adding payload
robot.links(6).m
robot.links(6).r

% Inertia matrix before adding payload
M0 = robot.inertia(Q(1,:));

% Add box
m_box = 10.0;

% Apply the combined gripper and box payload at the tool center offset
% Rough estimate of COM
payloadMass = m_gripper + m_box;
payloadOffset = [0 0 0.25];
robot.payload(payloadMass, payloadOffset);


% Inspect link 6 after adding payload
robot.links(6).m
robot.links(6).r

% Inertia matrix after adding payload
M1 = robot.inertia(Q(1,:));

disp('Change in inertia matrix:')
disp(M1 - M0)

% Required joint torques with payload
tau_payload = robot.rne(Q,QD,QDD);

% Additional torque caused by payload
tau_diff = tau_payload - tau;

% Plot effect of payload
figure;
tiledlayout(3,2);

for j = 1:6

    nexttile;

    plot(t,tau(:,j),'LineWidth',1.5);
    hold on;

    plot(t,tau_payload(:,j),'--','LineWidth',1.5);

    grid on;
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title(sprintf('Joint %d',j));

    legend('Without payload','With payload');

end

sgtitle('Effect of Payload on Joint Torques');

