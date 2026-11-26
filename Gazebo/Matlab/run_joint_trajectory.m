%% Denne filen kjører en 12-eskers palleteringsbane i Gazebo fra MATLAB over ROS 2.

%    Copyright:     HVL
%    Project:       ELE208 - Robotmanipulatorer
%    Author:        Christoffer Træet Herland
%    Date created:  Høst 2026
%    Input:         ROS 2 '/joint_states' og '/arm_controller/follow_joint_trajectory' (Gazebo på VM)
%
%    Output:        Leddbanekommandoer for 12 esker (2 lag x 6 posisjoner) i Gazebo
%---------------------------------------------------------------------%
%%
% Note: Følger strukturen fra 'lab2.m' samt bevegelsestider og baneplan fra
% 'InverseKinematics_Palletizing12boxes_v2.m' (5.0 s per syklus,
% 5.-gradspolynom på vertikale kartesiske bevegelser og 0.1 s vakuum-dwell).
% Verifiserer alle 26 veipunkter mot DH-foroverkinematikk og leddgrensene
% før kjøring.
% Utviklet med hjelp fra Gemini (AI).

%% struktur følger lab2 + InverseKinematics_Palletizing12boxes_v2.m (pallet_cycles.mat)
%
% VM side først:  cd ~/ros2_ws && source install/setup.bash
%                 export ROS_DOMAIN_ID=18
%                 ros2 launch palletizer_robot palletizer.launch.py
%
% Oppdatert mot nyeste git-versjon (Palletizing_Robotarm.m +
% InverseKinematics_Palletizing12boxes_v2.m + rapport Tabell 1):
%   - L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15], TCP = Tz(0.10) langs z6
%   - Konfigurasjonsgren innenfor leddgrenser i Tabell 1:
%       q1 in [70, 230] deg (78.5..197.6 deg), q5 = -90 deg
%   - 5.-gradspolynom (tpoly) på kartesiske vertikalbevegelser
%   - Bevegelsestider fra InverseKinematics_Palletizing12boxes_v2.m:
%       t1=0.4, tGrip=0.1, t2=0.4, t3=1.5, t4=0.5, tGrip=0.1, t5=0.5, t6=1.5 -> 5.0 s/syklus

%% Part A - Setup ROS 2 and action client
clc
clear

setenv("ROS_DOMAIN_ID", "18");
pallet_node = ros2node("/palletizer_client_node");

disp("ROS2 actions:")
ros2("action", "list");

joints = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};

actionType = "control_msgs/FollowJointTrajectory";
actionName = "/arm_controller/follow_joint_trajectory";

client = ros2actionclient(pallet_node, actionName, actionType);

disp("Waiting for FollowJointTrajectory action server ...");
ok = waitForServer(client);
if ~ok, error("Action server not reachable."); end
disp("Action server available.");

%% Part B - Waypoints & Box Order from InverseKinematics_Palletizing12boxes_v2.m
%   12 boxes = 2 layers of 6, placed back-to-front in Y (+0.4 -> 0.0 -> -0.4),
%   alternating far/near in X (1.0 -> 0.6):
%     boxX = [1.0, 0.6, 1.0, 0.6, 1.0, 0.6]
%     boxY = [0.4, 0.4, 0.0, 0.0, -0.4, -0.4]
%   Layer 1 (B1..B6): zPlace = 0.544, zAbove = max(1.0, zPlace+0.20) = 1.000
%   Layer 2 (B7..B12): zPlace = 0.944, zAbove = max(1.0, zPlace+0.20) = 1.144
boxX = [1.0, 0.6, 1.0, 0.6, 1.0, 0.6];
boxY = [0.4, 0.4, 0.0, 0.0, -0.4, -0.4];
palletHeight = 0.144;
boxHeight    = 0.400;

T = struct();
T.above_c = [0.0, -1.0, 1.000];
T.pickup  = [0.0, -1.0, 0.800];
for k = 1:12
    layer  = ceil(k / 6);
    posIdx = mod(k - 1, 6) + 1;
    zPlace = palletHeight + (layer - 1) * boxHeight + boxHeight;
    zAbove = max(1.000, zPlace + 0.200);
    T.(['above_B' num2str(k)]) = [boxX(posIdx), boxY(posIdx), zAbove];
    T.(['place_B' num2str(k)]) = [boxX(posIdx), boxY(posIdx), zPlace];
end

% Compute & verify all 26 waypoints against DH FK and Report Tabell 1 limits
% URDF theta limits matching Palletizing_Robotarm.m qlim (theta2=q2+pi/2, theta4=q4-pi/2):
jLower = [deg2rad(70),  0,    -pi, -3*pi/2, -pi, -pi];
jUpper = [deg2rad(230), pi,    pi,    pi/2,  pi,  pi];

Q = struct();
Rd = diag([1 -1 -1]);   % pose plasserer tool nedover på hver pose
for name = fieldnames(T)'
    n = name{1};
    Q.(n) = palletizer_ikine(T.(n));
    if any(Q.(n) < jLower - 1e-6) || any(Q.(n) > jUpper + 1e-6)
        error('%s violates Tabell 1 joint limits', n);
    end
    Mt = eye(4); Mt(1:3,1:3) = Rd; Mt(1:3,4) = T.(n).';
    e = norm(palletizer_fkine(Q.(n)) - Mt);
    if e > 1e-5, error('%s: FK residual %.3e - waypoint/target mismatch', n, e); end
end
disp("All 26 pick/place waypoints verified against DH chain (< 1e-5) and Tabell 1 limits.");

% Movement times [s] from InverseKinematics_Palletizing12boxes_v2.m (5.0 s total/cycle):
%   t1=0.4 (Start->Pickup), tGrip=0.1, t2=0.4 (Pickup->Start),
%   t3=1.5 (Start->Above pallet), t4=0.5 (Above->Place), tGrip=0.1,
%   t5=0.5 (Place->Above), t6=1.5 (Above->Start)
t1 = 0.4; t2 = 0.4; t3 = 1.5; t4 = 0.5; t5 = 0.5; t6 = 1.5;
tGrip  = 0.1;
T_init = 1.5;   % initial move to above_c
N_cart = 6;     % intermediate quintic Cartesian steps per vertical stroke

%% Part B2 - unwrap commands relative to the arm's CURRENT accumulated state
jsSub = ros2subscriber(pallet_node, "/joint_states", "sensor_msgs/JointState");
jsMsg = receive(jsSub, 10);
qref = extractOrderedJoints(jsMsg, joints);
disp("Current joint state (accumulated):"); disp(qref);

%% Part C - Move to above-conveyor, then 12 box cycles (2 layers of 6)
u1 = unwrapPt(Q.above_c, qref); qref = u1;
runTraj(client, joints, u1, T_init);

for k = 1:12
    layer = ceil(k / 6);
    fprintf("=== Box %d, Layer %d ===\n", k, layer);
    pA = T.(['above_B' num2str(k)]);
    pP = T.(['place_B' num2str(k)]);
    qA = Q.(['above_B' num2str(k)]);

    % Build cycle trajectory matching InverseKinematics_Palletizing12boxes_v2.m:
    %   1) tpoly cartesianMove: above_c -> pickup  (t1 = 0.4 s)
    %   2) vacuum ON dwell at pickup               (tGrip = 0.1 s)
    %   3) tpoly cartesianMove: pickup  -> above_c (t2 = 0.4 s)
    %   4) joint-space move:    above_c -> above_B (t3 = 1.5 s)
    %   5) tpoly cartesianMove: above_B -> place_B (t4 = 0.5 s)
    %   6) vacuum OFF dwell at place_B             (tGrip = 0.1 s)
    %   7) tpoly cartesianMove: place_B -> above_B (t5 = 0.5 s)
    %   8) joint-space return:  above_B -> above_c (t6 = 1.5 s) -> 5.0 s total
    q_down  = cartesianPath(T.above_c, T.pickup, N_cart);
    q_up    = cartesianPath(T.pickup, T.above_c, N_cart);
    q_place = cartesianPath(pA, pP, N_cart);
    q_lift  = cartesianPath(pP, pA, N_cart);

    segs = [q_down; q_down(end,:); q_up; qA; q_place; q_place(end,:); q_lift; Q.above_c];

    tt1 = linspace(0, t1, N_cart + 1);               tt1 = tt1(2:end);
    ttG1 = tt1(end) + tGrip;
    tt2 = ttG1 + linspace(0, t2, N_cart + 1);        tt2 = tt2(2:end);
    tt3 = tt2(end) + t3;
    tt4 = tt3 + linspace(0, t4, N_cart + 1);         tt4 = tt4(2:end);
    ttG2 = tt4(end) + tGrip;
    tt5 = ttG2 + linspace(0, t5, N_cart + 1);        tt5 = tt5(2:end);
    tt6 = tt5(end) + t6;
    t_vec = [tt1, ttG1, tt2, tt3, tt4, ttG2, tt5, tt6];

    U = zeros(size(segs));
    for i = 1:size(segs,1)
        U(i,:) = unwrapPt(segs(i,:), qref);
        qref = U(i,:);
    end
    runTraj(client, joints, U, t_vec);
    pause(0.1);
end

disp("Pallet is full!");

%% Local functions
function Qpath = cartesianPath(pStart, pEnd, N)
    % Straight-line Cartesian interpolation with 5th-order polynomial time scaling
    % (tpoly, matching InverseKinematics_Palletizing12boxes_v2.m & Report Sec 2.5).
    Qpath = zeros(N, 6);
    for i = 1:N
        u = i / N;
        s = 10*u^3 - 15*u^4 + 6*u^5;
        p = (1 - s) * pStart + s * pEnd;
        Qpath(i, :) = palletizer_ikine(p);
    end
end

function q = palletizer_ikine(p)
    % Exact closed-form IK for L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15],
    % tool = Tz(0.10), downward orientation Rd = diag([1, -1, -1]),
    % on the Palletizing_Robotarm.m / pallet_cycles.mat branch
    % (q1 in [70, 230] deg, q5 = -pi/2).
    L1 = 0.1; L2 = 0.9; L3 = 0.8; d4 = 0.2; d5 = 0.15;
    x = p(1); y = p(2); z = p(3);
    r0 = sqrt(x^2 + y^2 - d4^2);
    q1 = atan2(y, x) + pi - atan2(d4, r0);
    while q1 < deg2rad(70),  q1 = q1 + 2*pi; end
    while q1 > deg2rad(230), q1 = q1 - 2*pi; end
    r  = -(r0 - d5);
    zw = z + 0.25 - L1;
    c3 = max(-1, min(1, (r^2 + zw^2 - L2^2 - L3^2) / (2 * L2 * L3)));
    q3 = acos(c3);
    q2 = atan2(zw, r) - atan2(L3 * sin(q3), L2 + L3 * cos(q3));
    q4 = pi/2 - q2 - q3;
    q5 = -pi/2;
    q6 = q1 - pi/2;
    q  = [q1, q2, q3, q4, q5, q6];
end

function q = extractOrderedJoints(jsMsg, jointNames)
    q = zeros(1, numel(jointNames));
    for i = 1:numel(jointNames)
        idx = find(strcmp(jsMsg.name, jointNames{i}), 1);
        if ~isempty(idx)
            q(i) = jsMsg.position(idx);
        else
            q(i) = jsMsg.position(i);
        end
    end
end

function qc = unwrapPt(qw, qref)
    % nearest representation of waypoint qw relative to accumulated qref:
    % qc = qref + shortest angular difference (element-wise, wrap-aware).
    qc = qref + atan2(sin(qw - qref), cos(qw - qref));
end

function runTraj(client, joints, Q, tSec)
    traj = ros2message("trajectory_msgs/JointTrajectory");
    traj.joint_names = joints;
    traj.points = [];
    for i = 1:size(Q,1)
        traj.points = [traj.points, makePoint(Q(i,:), tSec(i))]; %#ok<AGROW>
    end
    goalMsg = ros2message(client);
    goalMsg.trajectory = traj;
    goalMsg.goal_time_tolerance.sec     = int32(0);
    goalMsg.goal_time_tolerance.nanosec = uint32(500000000);  % 0.5 s
    goalHandle = sendGoal(client, goalMsg);
    while true
        exStatus = getStatus(client, goalHandle);
        if exStatus == 4 || exStatus == 5 || exStatus == 6
            break;
        end
        pause(0.05);
    end
    resultMsg = getResult(client, goalHandle);
    if exStatus ~= 4
        disp(resultMsg);
        error("Trajectory failed with status %d", exStatus);
    end
    fprintf("  trajectory OK (%d points, %.2f s)\n", size(Q,1), tSec(end));
end

function pt = makePoint(positions, tSec)
    pt = ros2message("trajectory_msgs/JointTrajectoryPoint");
    pt.positions     = positions;
    pt.velocities    = zeros(1, numel(positions));
    pt.accelerations = zeros(1, numel(positions));
    pt.effort        = [];
    secFloor = floor(tSec);
    pt.time_from_start.sec     = int32(secFloor);
    pt.time_from_start.nanosec = uint32(round((tSec - secFloor) * 1e9));
end

function T = dhcol(d, a, al)
    % A_j with theta = 0:  Tz(d)*Tx(a)*Rx(al)
    ca = cos(al); sa = sin(al);
    T = eye(4);
    T(1:3,1:3) = [1 0 0; 0 ca -sa; 0 sa ca];
    T(1:3,4)   = [a; 0; d];
end

function M = palletizer_fkine(q)
    % 12box link lengths, standard DH, tool = Tz(0.1) along z6 (report eq. 3).
    L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15];
    dj = [L(1), 0, 0, L(4), L(5), L(6)];
    aj = [0, L(2), L(3), 0, 0, 0];
    al = [pi/2, 0, 0, -pi/2, pi/2, 0];
    M = eye(4);
    for j = 1:6
        Rz = [cos(q(j)) -sin(q(j)) 0 0; sin(q(j)) cos(q(j)) 0 0; 0 0 1 0; 0 0 0 1];
        M = M * Rz * dhcol(dj(j), aj(j), al(j));
    end
    M = M * [eye(3), [0;0;0.1]; 0 0 0 1];
end
