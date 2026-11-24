%% Denne filen .  

% % 
%    Copyright:     HVL
%    Project:	    ELE208 - Robotmanipulatorer
%    Author:        Erlend Østbø
%    Co author:     Ingrid Vikane (smoothing av kartesisk bane vha. tpoly, samt fikk ikine til å fungere i cartesian movement
%                   fjernet animeringen og la det inn i en egen fil,
%                   implementerte lagring av banene til en egen fil vha. AI) 
%    Date created:  Høst 2026
%    Input: 'Workspace.m'
%           
%    Output: 'pallet_cycles.mat'
%---------------------------------------------------------------------%
%%
% Note: 
%

close all; 
clc;

%% Load robot and workspace
run('Workspace.m');

% Joint limits are defined in Palletizing_Robotarm.m
rng(1);

%% Settings
N = 100;                 % Points per movement
% Movement times [s]

t1 = 0.4;       % Start -> Pickup
t2 = 0.4;       % Pickup -> Start
t3 = 1.5;       % Start -> Above pallet
t4 = 0.5;       % Above pallet -> Place
t5 = 0.5;       % Place -> Above pallet
t6 = 1.5;       % Above pallet -> Start

tGrip = 0.1;    % Vakuum på og av

movementTimes = [t1 t2 t3 t4 t5 t6 tGrip];

% Box settings
numBoxes = 12;           % number of boxes
numPositions = 6;

posTolerance = 0.01;     % 1 cm toleranse
rotTolerance = 0.05;
maxJointStep = deg2rad(20);

% Original pallet coordinates
boxX = [1.0 0.6 1.0 0.6 1.0 0.6];
boxY = [0.4 0.4 0.0 0.0 -0.4 -0.4];

% Simulated box dimension
boxHeight = 0.40;

% Robot orientation
R = trotx(pi);

% Conveyor positions
T_above_c = transl(0,-1.0,1.0)*R;
T_pickup  = transl(0,-1.0,0.8)*R;

%% Initial IK
q0 = deg2rad([80 5 70 15 -90 -10]);

q_above_c = solveIK(Palletizing,T_above_c,q0);
q_pickup = solveIK(Palletizing,T_pickup,q_above_c);


currentQ = q_above_c;

% Track height at each pallet position
stackHeight = zeros(1,numPositions);

% Track height at each pallet position
stackHeight = zeros(1,numPositions);

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

    cyc = struct('t', [], 'q', [], 'qd', [], 'qdd', []);

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

    cyc = appendSegment(cyc, q_down, t1); 

    % Simulated gripper closes maybe flag for later
    fprintf('Gripper CLOSED\n');
    cyc = appendSegment(cyc, repmat(q_down(end,:), 5, 1), tGrip); 

    % Lift box
    q_up = flipud(q_down);
    cyc = appendSegment(cyc, q_up, t2);     

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

    cyc = appendSegment(cyc, q_move, t3);

    %% Place box

    q_place = cartesianMove( ...
        Palletizing,q_above_B,T_above_B,T_B,N);

    cyc = appendSegment(cyc, q_place, t4); 

    % Simulated gripper opens
    fprintf('Gripper OPEN\n');
    cyc = appendSegment(cyc, repmat(q_place(end,:), 5, 1), tGrip);

    %% Update pallet stack

    stackHeight(positionIndex) = ...
        stackHeight(positionIndex)+sensor.height;

    

    %% Lift gripper

    q_lift = flipud(q_place);
    cyc = appendSegment(cyc, q_lift, t5);

    %% Return to conveyor

    q_return = jtraj( ...
        q_above_B,q_above_c,N);

    validatePath(Palletizing,q_return,maxJointStep);

    cyc = appendSegment(cyc, q_return, t6); 

    currentQ = q_above_c;

    fprintf('Ready for next box\n');

    % Hastighet og akselerasjon for hele syklusen
    [~, cyc.qd]  = gradient(cyc.q,  1, cyc.t);
    [~, cyc.qdd] = gradient(cyc.qd, 1, cyc.t);

    cycles(boxNumber) = cyc;  

end

disp('Pallet is full!');

%% Lagre alle banene
save('pallet_cycles.mat', 'cycles');
fprintf('Lagret %d sykluser i pallet_cycles.mat\n', numel(cycles));

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

    s = tpoly(0, 1, N);             % tpoly: gir et 5.gradspolynom for å unngå hopp i fart ved start og slutt. 

    for i = 2:N

        p = (1-s(i))*pA + s(i)*pB;

        T = transl(p)*r2t(R);

        % Invers Kinematikk: forrige punkt brukes som startgjetning. 
        qNew = robot.ikine(T, 'q0', qPath(i-1,:));

        % Fikk problemer ved bruk av ikine. Brukte originalt angdiff 
        % qPath(i,:) = qPath(i-1,:) + angdiff(qNew, qPath(i-1,:));

        % angdiff() for matlab og toolbox er forskjellige, så utifra hvilken 
        % som er øverst i stien vil koden feile.
        % Dermed blir det heller regnet ut slik: 
        dq = qNew - qPath(i-1,:);
        dq = mod(dq + pi, 2*pi) - pi;
        qPath(i,:) = qPath(i-1,:) + dq;


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

%% -------------------------------------------------------------------------%%
% Lagrer banene som segmenter. Funksjon for å lagre banene er laget ved hjelp av ChatGPT. 
% Brukes slik at 'differensialkinematic.m', 'Dynamics_and_control.m' og 'Animasjon.m' ikke må
% kjøre hele dette programmet hver gang koden skal testes. 

function cyc = appendSegment(cyc, qSeg, T)
% Legger et bevegelsessegment (qSeg, varighet T [s]) til slutten av banen cyc.
    n    = size(qSeg, 1);
    tSeg = linspace(0, T, n)';

    if isempty(cyc.q)
        cyc.q = qSeg;
        cyc.t = tSeg;
    else
        cyc.q = [cyc.q; qSeg(2:end,:)];
        cyc.t = [cyc.t; cyc.t(end) + tSeg(2:end)];
    end
end

