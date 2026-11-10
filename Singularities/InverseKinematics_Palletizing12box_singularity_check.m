
clc;
clear all;
close all;

%% Load robot and workspace
%run('Palletizing_Robotarm.m');
run('Workspace.m');

% Joint limits are defined in Palletizing_Robotarm.m
rng(1);

%% Settings
N = 15;                  % Points per movement
numBoxes = 12;           % number of boxes
numPositions = 6;

posTolerance = 0.01;     % 1 cm toleranse
rotTolerance = 0.05;
maxJointStep = deg2rad(20);

% Original pallet coordinates
boxX = [1.0 0.6 1.0 0.6 1.0 0.6];
boxY = [0.4 0.4 0.0 0.0 -0.4 -0.4];

% Simulated box dimension
boxHeight = 0.4;

% Robot orientation
R = trotx(pi);

% Conveyor positions
T_above_c = transl(0,-1.0,1.0)*R;
T_pickup  = transl(0,-1.0,0.8)*R;

%% Initial IK
q0 = deg2rad([-90 -30 60 0 30 0]);

q_above_c = solveIK(Palletizing,T_above_c,q0);
q_pickup = solveIK(Palletizing,T_pickup,q_above_c);

%% Initialize animation
fig = findobj('Type','figure', ...
    'Name','6DOF Palletizing Workspace');

if isempty(fig)
    fig = figure;
else
    fig = fig(1);
end

figure(fig);
Palletizing.plot(q_above_c);
drawnow;

currentQ = q_above_c;

% Track height at each pallet position
stackHeight = zeros(1,numPositions);

video = VideoWriter('Palletizing_12Boxes.mp4','MPEG-4');
video.FrameRate = 25;
open(video);

%% Main loop - one box at a time

for boxNumber = 1:numBoxes

    layer = ceil(boxNumber/numPositions);
    positionIndex = mod(boxNumber-1,numPositions)+1;

    fprintf('\n========================\n');
    fprintf('Box %d, Layer %d\n',boxNumber,layer);

    %% Simulated sensor

    sensor.boxDetected = true;
    sensor.height = boxHeight;
    sensor.angle = 0;

    if ~sensor.boxDetected
        continue;
    end

    fprintf('Box detected\n');
    fprintf('Height: %.3f m\n',sensor.height);
    fprintf('Angle: %.1f deg\n',rad2deg(sensor.angle));

    %% Pickup

    fprintf('Moving to pickup\n');

    q_down = cartesianMove( ...
        Palletizing,q_above_c,T_above_c,T_pickup,N);

    animatePath(Palletizing,q_down,fig,video);

    % Simulated gripper closes maybe flag for later
    fprintf('Gripper CLOSED\n');
    pause(0.15);

    % Lift box
    q_up = flipud(q_down);
    animatePath(Palletizing,q_up,fig,video);

    currentQ = q_above_c;

    %% Calculate pallet position

    x = boxX(positionIndex);
    y = boxY(positionIndex);

    % Height of the box stack at this position
    zPlace = pallet.height + ...
        stackHeight(positionIndex) + sensor.height;

    % Clearance above placement position
    zAbove = max(1.0,zPlace+0.20);

    T_above_B = transl(x,y,zAbove)*R;
    T_B = transl(x,y,zPlace)*R;

    %% Calculate Inverse kinematic

    fprintf('Calculating pallet IK\n');

    q_above_B = solveIK( ...
        Palletizing,T_above_B,currentQ);

    fprintf('Base rotation: %.2f deg\n', ...
        rad2deg(q_above_B(1)));

    %% Move to pallet

    q_move = jtraj(currentQ,q_above_B,N);

    validatePath(Palletizing,q_move,maxJointStep);

    checkSingularity(Palletizing, q_move, 'q_move');

    validatePath(Palletizing,q_move,maxJointStep);

    animatePath(Palletizing,q_move,fig,video);

    %% Place box

    q_place = cartesianMove( ...
        Palletizing,q_above_B,T_above_B,T_B,N);

    checkSingularity(Palletizing, q_place, 'q_place');

    animatePath(Palletizing,q_place,fig,video);

    % Simulated gripper opens
    fprintf('Gripper OPEN\n');
    pause(0.15);

    %% Update pallet stack

    stackHeight(positionIndex) = ...
        stackHeight(positionIndex)+sensor.height;

    

    %% Lift gripper

    q_lift = flipud(q_place);
    animatePath(Palletizing,q_lift,fig,video);

    %% Return to conveyor

    q_return = jtraj( ...
        q_above_B,q_above_c,N);

    validatePath(Palletizing,q_return,maxJointStep);

    checkSingularity(Palletizing, q_return, 'q_return');

    animatePath(Palletizing,q_return,fig,video);

    currentQ = q_above_c;

    fprintf('Ready for next box\n');

    % Simulated wait before next box
    pause(0.1);

end

disp('Pallet is full!');
%% Save and close video
close(video);
disp('Video saved: Palletizing_12Boxes.mp4');

%% =========================================
% INVERSE KINEMATICS
% =========================================

function qBest = solveIK(robot,Ttarget,qSeed)

    lower = robot.qlim(:,1)';
    upper = robot.qlim(:,2)';

    bestCost = inf;
    qBest = [];

    desired = transl(Ttarget);
    Rdesired = t2r(Ttarget);

    for attempt = 1:100

        if attempt == 1
            seed = qSeed;
        else
            seed = lower + ...
                rand(1,6).*(upper-lower);
        end

        qTest = robot.ikcon(Ttarget,seed);

        Tactual = robot.fkine(qTest);

        actual = transl(Tactual);
        Ractual = t2r(Tactual);

        posError = norm( ...
            desired(:)-actual(:));

        rotError = norm( ...
            Rdesired-Ractual,'fro');

        validLimits = all( ...
            qTest >= lower-1e-6 & ...
            qTest <= upper+1e-6);

        if validLimits && ...
           posError < 0.01 && ...
           rotError < 0.05

            cost = norm(qTest-qSeed);

            if cost < bestCost
                bestCost = cost;
                qBest = qTest;
            end
        end
    end

    if isempty(qBest)
        error('No valid IK solution found');
    end

end

%% =========================================
% CARTESIAN MOVEMENT
% =========================================

function qPath = cartesianMove(robot,qStart,TA,TB,N)

    pA = transl(TA);
    pB = transl(TB);

    pA = pA(:)';
    pB = pB(:)';

    R = t2r(TA);

    qPath = zeros(N,6);
    qPath(1,:) = qStart;

    for i = 2:N

        s = (i-1)/(N-1);

        p = (1-s)*pA+s*pB;

        T = transl(p)*r2t(R);

        qPath(i,:) = robot.ikcon( ...
            T,qPath(i-1,:));

        Tactual = robot.fkine(qPath(i,:));

        desired = transl(T);
        actual = transl(Tactual);

        posError = norm( ...
            desired(:)-actual(:));

        rotError = norm( ...
            t2r(T)-t2r(Tactual),'fro');

        lower = robot.qlim(:,1)';
        upper = robot.qlim(:,2)';

        validLimits = all( ...
            qPath(i,:) >= lower-1e-6 & ...
            qPath(i,:) <= upper+1e-6);

        if ~validLimits || ...
           posError > 0.01 || ...
           rotError > 0.05
            error('Invalid Cartesian IK at step %d',i);
        end
    end

end


%% =========================================
% VALIDATE JOINT TRAJECTORY
% =========================================

function validatePath(robot,qPath,maxStep)

lower = robot.qlim(:,1)';
upper = robot.qlim(:,2)';

if any(any(qPath < lower-1e-6 | ...
        qPath > upper+1e-6))
    error('Joint limits violated');
end

jointChanges = abs(diff(qPath,1,1));

if any(jointChanges(:) > maxStep)
    error('Large joint jump detected');
end

end



%% =========================================
% ANIMATE ROBOT
% =========================================

function animatePath(robot,qPath,fig,video)

    figure(fig);

    for i = 1:size(qPath,1)

        robot.animate(qPath(i,:));

        drawnow limitrate;

        % Record video
        frame = getframe(fig);
        writeVideo(video,frame);

    end

end


%% =========================================
% Check Singularity
% =========================================

function checkSingularity(robot, qPath, label)

if nargin < 3
    label = 'trajectory';
end

if size(qPath, 1) < 1
    return;
end

detThreshold = 1e-5;
worstAbsDet = inf;
worstRank = inf;

for i = 1:size(qPath, 1)

    q = qPath(i, :);
    J = robot.jacob0(q);

    if size(J, 1) ~= size(J, 2)
        error('Jacobian must be square to use det().');
    end

    detJ = det(J);
    absDetJ = abs(detJ);
    r = rank(J);

    worstAbsDet = min(worstAbsDet, absDetJ);
    worstRank = min(worstRank, r);

    if r < 6 || absDetJ < detThreshold
        error('%s is near singular at step %d (rank=%d, det=%.3e)', ...
            label, i, r, detJ);
    end

end

fprintf('%s: no singularity detected, worst rank = %d, worst |det(J)| = %.3e\n', ...
    label, worstRank, worstAbsDet);

end
