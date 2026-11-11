%% struktur følger lab2 + InverseKinematics_Palletizing12boxes.m
%
% VM side først:  cd ~/ros2_ws && source install/setup.bash
%                 ros2 launch palletizer_robot palletizer.launch.py
%

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

%% Part B - Waypoints & Box Order from InverseKinematics_Palletizing12boxes.m
%   L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15].
%   12 boxes = 2 layers of 6, placed back-to-front in Y (+0.4 -> 0.0 -> -0.4),
%   alternating far/near in X (1.0 -> 0.6):
%     boxX = [1.0, 0.6, 1.0, 0.6, 1.0, 0.6]
%     boxY = [0.4, 0.4, 0.0, 0.0, -0.4, -0.4]
%   Layer 1 (B1..B6): zPlace = 0.544, zAbove = max(1.0, zPlace+0.20) = 1.000
%   Layer 2 (B7..B12): zPlace = 0.944, zAbove = max(1.0, zPlace+0.20) = 1.144
Q = struct();
Q.home      = [0 0 0 0 0 0];
Q.above_c   = [-1.369438  1.492066 -1.170571 -1.892291  1.570796  0.201358];
Q.pickup    = [-1.369438  1.535907 -1.472680 -1.634023  1.570796  0.201358];

% Layer 1 (B1..B6) matching boxX/boxY order in InverseKinematics_Palletizing12boxes.m
Q.above_B1  = [ 0.567286  1.400204 -1.065140 -1.905861  1.570796  2.138082]; % (1.0,  0.4, 1.000)
Q.above_B2  = [ 0.869038  1.804710 -1.454455 -1.921051  1.570796  2.439834]; % (0.6,  0.4, 1.000)
Q.above_B3  = [ 0.201358  1.492066 -1.170571 -1.892291  1.570796  1.772154]; % (1.0,  0.0, 1.000)
Q.above_B4  = [ 0.339837  1.936672 -1.539337 -1.968132  1.570796  1.910633]; % (0.6,  0.0, 1.000)
Q.above_B5  = [-0.193727  1.400204 -1.065140 -1.905861  1.570796  1.377069]; % (1.0, -0.4, 1.000)
Q.above_B6  = [-0.306968  1.804710 -1.454455 -1.921051  1.570796  1.263829]; % (0.6, -0.4, 1.000)

% Layer 2 (B7..B12) matching boxX/boxY order in InverseKinematics_Palletizing12boxes.m
Q.above_B7  = [ 0.567286  1.312607 -0.754255 -2.129148  1.570796  2.138082]; % (1.0,  0.4, 1.144)
Q.above_B8  = [ 0.869038  1.734301 -1.202015 -2.103082  1.570796  2.439834]; % (0.6,  0.4, 1.144)
Q.above_B9  = [ 0.201358  1.414779 -0.884048 -2.101528  1.570796  1.772154]; % (1.0,  0.0, 1.144)
Q.above_B10 = [ 0.339837  1.861340 -1.291318 -2.140818  1.570796  1.910633]; % (0.6,  0.0, 1.144)
Q.above_B11 = [-0.193727  1.312607 -0.754255 -2.129148  1.570796  1.377069]; % (1.0, -0.4, 1.144)
Q.above_B12 = [-0.306968  1.734301 -1.202015 -2.103082  1.570796  1.263829]; % (0.6, -0.4, 1.144)

% Place poses for Layer 1 (B1..B6)
Q.place_B1  = [ 0.567286  1.422787 -1.670513 -1.323070  1.570796  2.138082]; % (1.0,  0.4, 0.544)
Q.place_B2  = [ 0.869038  1.838469 -2.057658 -1.351608  1.570796  2.439834]; % (0.6,  0.4, 0.544)
Q.place_B3  = [ 0.201358  1.508194 -1.766348 -1.312642  1.570796  1.772154]; % (1.0,  0.0, 0.544)
Q.place_B4  = [ 0.339837  2.000074 -2.156132 -1.414739  1.570796  1.910633]; % (0.6,  0.0, 0.544)
Q.place_B5  = [-0.193727  1.422787 -1.670513 -1.323070  1.570796  1.377069]; % (1.0, -0.4, 0.544)
Q.place_B6  = [-0.306968  1.838469 -2.057658 -1.351608  1.570796  1.263829]; % (0.6, -0.4, 0.544)

% Place poses for Layer 2 (B7..B12)
Q.place_B7  = [ 0.567286  1.420467 -1.162425 -1.828838  1.570796  2.138082]; % (1.0,  0.4, 0.944)
Q.place_B8  = [ 0.869038  1.824109 -1.541980 -1.852926  1.570796  2.439834]; % (0.6,  0.4, 0.944)
Q.place_B9  = [ 0.201358  1.510668 -1.263630 -1.817834  1.570796  1.772154]; % (1.0,  0.0, 0.944)
Q.place_B10 = [ 0.339837  1.958872 -1.626638 -1.903031  1.570796  1.910633]; % (0.6,  0.0, 0.944)
Q.place_B11 = [-0.193727  1.420467 -1.162425 -1.828838  1.570796  1.377069]; % (1.0, -0.4, 0.944)
Q.place_B12 = [-0.306968  1.824109 -1.541980 -1.852926  1.570796  1.263829]; % (0.6, -0.4, 0.944)

% Target TCP poses [x y z] (trotx(pi) orientation for all).
T = struct();
T.above_c   = [ 0  -1.0  1.000];   T.pickup    = [ 0  -1.0  0.800];
T.above_B1  = [1.0  0.4  1.000];   T.place_B1  = [1.0  0.4  0.544];
T.above_B2  = [0.6  0.4  1.000];   T.place_B2  = [0.6  0.4  0.544];
T.above_B3  = [1.0  0.0  1.000];   T.place_B3  = [1.0  0.0  0.544];
T.above_B4  = [0.6  0.0  1.000];   T.place_B4  = [0.6  0.0  0.544];
T.above_B5  = [1.0 -0.4  1.000];   T.place_B5  = [1.0 -0.4  0.544];
T.above_B6  = [0.6 -0.4  1.000];   T.place_B6  = [0.6 -0.4  0.544];
T.above_B7  = [1.0  0.4  1.144];   T.place_B7  = [1.0  0.4  0.944];
T.above_B8  = [0.6  0.4  1.144];   T.place_B8  = [0.6  0.4  0.944];
T.above_B9  = [1.0  0.0  1.144];   T.place_B9  = [1.0  0.0  0.944];
T.above_B10 = [0.6  0.0  1.144];   T.place_B10 = [0.6  0.0  0.944];
T.above_B11 = [1.0 -0.4  1.144];   T.place_B11 = [1.0 -0.4  0.944];
T.above_B12 = [0.6 -0.4  1.144];   T.place_B12 = [0.6 -0.4  0.944];

Rd = diag([1 -1 -1]);   % pose plasserer tool nedover på hver pose
for name = fieldnames(T)'
    n = name{1};
    if any(abs(Q.(n)) > pi), error('%s violates joint limits +/-pi', n); end
    Mt = eye(4); Mt(1:3,1:3) = Rd; Mt(1:3,4) = T.(n).';
    e = norm(palletizer_fkine(Q.(n)) - Mt);
    if e > 1e-5, error('%s: FK residual %.3e - waypoint/target mismatch', n, e); end
end
disp("All 26 pick/place waypoints verified against the DH chain (< 1e-5, full pose incl. tool).");

% Movement times [s] from InverseKinematics_Palletizing12boxes.m (5.0 s total per cycle):
%   t1=0.5 (Start->Pickup), t2=0.5 (Pickup->Start), t3=1.5 (Start->Above pallet),
%   t4=0.5 (Above pallet->Place), t5=0.5 (Place->Above pallet), t6=1.5 (Above pallet->Start)
movementTimes = [0.5, 0.5, 1.5, 0.5, 0.5, 1.5];
T_init = 1.5;   % initial move to above_c
N_cart = 5;     % intermediate Cartesian steps per vertical stroke (cartesianMove)

%% Part B2 - unwrap commands relative to the arm's CURRENT accumulated state
%   gz bullet-featherstone does NOT enforce the URDF joint limits, so joint
%   positions accumulate unboundedly and JTC interpolates from the accumulated
%   state: a wrapped command on the far side of the wrap makes the joint spin
%   multiple revolutions (observed: joint_6 ratcheted to -4800 rad). Every
%   commanded point is therefore made CONTINUOUS with the previous command
%   (shortest angular step), starting from the live /joint_states position.
jsSub = ros2subscriber(pallet_node, "/joint_states");
jsMsg = receive(jsSub, 10);
qref = reshape(jsMsg.position, 1, []);  % live accumulated positions (1x6)
assert(numel(qref) == 6, "expected 6 joint positions, got %d", numel(qref));
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

    % Build cycle trajectory matching InverseKinematics_Palletizing12boxes.m:
    %   1) cartesianMove: above_c -> pickup  (t1 = 0.5 s)
    %   2) cartesianMove: pickup  -> above_c (t2 = 0.5 s)
    %   3) jtraj:         above_c -> above_B (t3 = 1.5 s)
    %   4) cartesianMove: above_B -> place_B (t4 = 0.5 s)
    %   5) cartesianMove: place_B -> above_B (t5 = 0.5 s)
    %   6) jtraj:         above_B -> above_c (t6 = 1.5 s)
    q_down  = cartesianPath(T.above_c, T.pickup, N_cart);
    q_up    = flipud(q_down(1:end-1, :));
    q_place = cartesianPath(pA, pP, N_cart);
    q_lift  = flipud(q_place(1:end-1, :));

    segs = [q_down; q_up; qA; q_place; q_lift; Q.above_c];

    t1 = linspace(0, movementTimes(1), N_cart + 1); t1 = t1(2:end);
    t2 = movementTimes(1) + linspace(0, movementTimes(2), N_cart + 1); t2 = t2(2:end);
    t3 = sum(movementTimes(1:3));
    t4 = t3 + linspace(0, movementTimes(4), N_cart + 1); t4 = t4(2:end);
    t5 = t3 + movementTimes(4) + linspace(0, movementTimes(5), N_cart + 1); t5 = t5(2:end);
    t6 = sum(movementTimes);
    t_vec = [t1, t2, t3, t4, t5, t6];

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
    % Straight-line Cartesian interpolation from pStart to pEnd (excluding
    % pStart, including pEnd) solved via exact closed-form DH IK.
    Qpath = zeros(N, 6);
    for i = 1:N
        s = i / N;
        p = (1 - s) * pStart + s * pEnd;
        Qpath(i, :) = palletizer_ikine(p);
    end
end

function q = palletizer_ikine(p)
    % Exact closed-form IK for L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15],
    % tool = Tz(0.10), downward orientation Rd = diag([1, -1, -1]).
    L1 = 0.1; L2 = 0.9; L3 = 0.8; d4 = 0.2; d5 = 0.15;
    x = p(1); y = p(2); z = p(3);
    r0 = sqrt(x^2 + y^2 - d4^2);
    q1 = atan2(y, x) + atan2(d4, r0);
    r  = r0 - d5;
    zw = z + 0.25 - L1;
    c3 = max(-1, min(1, (r^2 + zw^2 - L2^2 - L3^2) / (2 * L2 * L3)));
    q3 = -acos(c3);
    q2 = atan2(zw, r) - atan2(L3 * sin(q3), L2 + L3 * cos(q3));
    q4 = -pi/2 - q2 - q3;
    q5 = pi/2;
    q6 = q1 + pi/2;
    q  = [q1, q2, q3, q4, q5, q6];
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
    fprintf("  trajectory OK (%d points)\n", size(Q,1));
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
    % 12box link lengths, L2/L3 switched 10 Nov 2026 (Palletizing_Robotarm.m),
    % standard DH, tool = Tz(0.1) along z6 (report eq. 2).
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
