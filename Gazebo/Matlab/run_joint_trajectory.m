%% struktur følger lab2
%
% VM side først:  cd ~/ros2_ws && source install/setup.bash
%                 ros2 launch palletizer_robot palletizer.launch.py
%
% denne filen følger bare
% banen akkurat nå (plukker ikke opp ved hjelp av gripper)
% må plotte for 12 bokser.
% kamera er i vm (ikke i dette scriptet)

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

%% Part B - Waypoints: IK of their pick/place poses
%   above_c -> pickup -> above_c -> above_Bn -> place_Bn -> above_Bn -> above_c
Q = struct();
Q.home     = [0 0 0 0 0 0];
Q.above_c  = [-1.266104  1.461707 -0.928932 -2.103572  1.570796  0.304693];
Q.pickup   = [-1.266104  1.555362 -1.387752 -1.738405  1.570796  0.304693];
Q.above_B1 = [-0.158933  1.821652 -1.249155 -2.143294  1.570796  1.411864];
Q.place_B1 = [-0.158933  1.935420 -2.108629 -1.397588  1.570796  1.411864];
Q.above_B2 = [ 0.523599  1.967168 -1.319440 -2.218524  1.570796  2.094395];
Q.place_B2 = [ 0.523599  2.137658 -2.189135 -1.519320  1.570796  2.094395];
Q.above_B3 = [ 1.017072  1.821652 -1.249155 -2.143294  1.570796  2.587869];
Q.place_B3 = [ 1.017072  1.935420 -2.108629 -1.397588  1.570796  2.587869];
Q.above_B4 = [-0.098230  1.348066 -0.785999 -2.132863  1.570796  1.472567];
Q.place_B4 = [-0.098230  1.441333 -1.692817 -1.319312  1.570796  1.472567];
Q.above_B5 = [ 0.304693  1.461707 -0.928932 -2.103571  1.570796  1.875489];
Q.place_B5 = [ 0.304693  1.540200 -1.802576 -1.308420  1.570796  1.875489];
Q.above_B6 = [ 0.662783  0.680376  0.785999 -3.037171  1.570796  2.233579];
Q.place_B6 = [ 0.662783  0.068645  1.692817  2.950926  1.570796  2.233579];

% Target TCP poses [x y z] (trotx(pi) orientasjon for alle).
T = struct();
T.above_c  = [ 0  -1.0  1.000];   T.pickup   = [ 0  -1.0  0.800];
T.above_B1 = [0.6 -0.4  1.000];   T.place_B1 = [0.6 -0.4  0.544];
T.above_B2 = [0.6  0.0  1.000];   T.place_B2 = [0.6  0.0  0.544];
T.above_B3 = [0.6  0.4  1.000];   T.place_B3 = [0.6  0.4  0.544];
T.above_B4 = [1.0 -0.4  1.000];   T.place_B4 = [1.0 -0.4  0.544];
T.above_B5 = [1.0  0.0  1.000];   T.place_B5 = [1.0  0.0  0.544];
T.above_B6 = [1.0  0.4  1.000];   T.place_B6 = [1.0  0.4  0.544];

Rd = diag([1 -1 -1]);   % pose plasserer tool nedover på hver pose
for name = fieldnames(T)'
    n = name{1};
    if any(abs(Q.(n)) > pi), error('%s violates joint limits +/-pi', n); end
    Mt = eye(4); Mt(1:3,1:3) = Rd; Mt(1:3,4) = T.(n).';
    e = norm(palletizer_fkine(Q.(n)) - Mt);
    if e > 1e-5, error('%s: FK residual %.3e - waypoint/target mismatch', n, e); end
end
disp("All 14 pick/place waypoints verified against the DH chain (< 1e-5, full pose incl. tool).");

T_move = 3;   %  sekunder per segment (jtraj T_move)

%% Part C - One check -> så seks boks poses
% Move to above-conveyor

runTraj(client, joints, [Q.above_c], T_move*ones(1,1));

for k = 1:6
    fprintf("=== box B%d cycle ===\n", k);
    qA = Q.(['above_B' num2str(k)]);
    qP = Q.(['place_B'  num2str(k)]);
    runTraj(client, joints, ...
        [Q.pickup; Q.above_c; qA; qP; qA; Q.above_c], ...
        T_move*(1:6));
    pause(1);
end

% End parked over pall, så home.
runTraj(client, joints, [Q.home], T_move);
disp("Palletizing sequence done.");

%% Local functions
function runTraj(client, joints, Q, tSec)
    traj = ros2message("trajectory_msgs/JointTrajectory");
    traj.joint_names = joints;
    traj.points = [];
    for i = 1:size(Q,1)
        traj.points = [traj.points, makePoint(Q(i,:), tSec(i))];
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
        pause(0.2);
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
    pt.time_from_start.sec     = int32(tSec);
    pt.time_from_start.nanosec = uint32(0);
end

function T = dhcol(d, a, al)
    % A_j with theta = 0:  Tz(d)*Tx(a)*Rx(al)
    ca = cos(al); sa = sin(al);
    T = eye(4);
    T(1:3,1:3) = [1 0 0; 0 ca -sa; 0 sa ca];
    T(1:3,4)   = [a; 0; d];
end

function M = palletizer_fkine(q)
    % Tabell 2/4 (standard DH), tool = Tz(0.1) along z6 (report eq. 2).
    L = [0.3, 0.8, 0.6, 0.3, 0.35, 0.3];
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

