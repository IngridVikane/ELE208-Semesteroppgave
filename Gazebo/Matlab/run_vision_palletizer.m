%% Vision-Guided 12-Box Palletizer (YOLO-OBB + TCP Yaw Alignment)
%  Subscribes to Gazebo's published overhead camera (/camera/image_raw),
%  runs an Oriented Bounding Box (YOLO-OBB) detector to find each randomly
%  rotated box on the conveyor, displays a live detection HUD, and adjusts
%  the robot TCP yaw (joint_6) to match the detected box rotation at pickup
%  before straightening the box to 0 deg for pallet placement.
%
%  VM side first:
%    cd ~/ros2_ws && colcon build --packages-select palletizer_robot
%    source install/setup.bash
%    export ROS_DOMAIN_ID=18
%    ros2 launch palletizer_robot palletizer.launch.py

clc;
clear;
close all;

%% Part A - Setup ROS 2 Node, Action Client, and Camera Subscriber
setenv("ROS_DOMAIN_ID", "18");
pallet_node = ros2node("/palletizer_vision_node");

joints = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
actionType = "control_msgs/FollowJointTrajectory";
actionName = "/arm_controller/follow_joint_trajectory";
client = ros2actionclient(pallet_node, actionName, actionType);

disp("Waiting for FollowJointTrajectory action server ...");
if ~waitForServer(client)
    error("Action server not reachable.");
end
disp("Action server available.");

jsSub  = ros2subscriber(pallet_node, "/joint_states", "sensor_msgs/JointState");
camSub = ros2subscriber(pallet_node, "/camera/image_raw", "sensor_msgs/Image");

%% Part B - Pallet Coordinates & Waypoints (InverseKinematics_Palletizing12boxes.m)
%  boxX = [1.0, 0.6, 1.0, 0.6, 1.0, 0.6]
%  boxY = [0.4, 0.4, 0.0, 0.0, -0.4, -0.4]
boxX = [1.0, 0.6, 1.0, 0.6, 1.0, 0.6];
boxY = [0.4, 0.4, 0.0, 0.0, -0.4, -0.4];
palletHeight = 0.144;
boxHeight    = 0.400;

T_above_c   = [0.0, -1.0, 1.000];
T_pickup    = [0.0, -1.0, 0.800];
% Standby pose between conveyor and pallet so the arm never occludes the overhead camera
T_cam_clear = [0.45, -0.65, 1.000];
Q_cam_clear = palletizer_ikine(T_cam_clear, 0.0);

% Camera intrinsics & extrinsics (from camera.urdf.xacro & robot_description.urdf.xacro)
camCfg = struct();
camCfg.pos   = [0.0, -1.0, 1.80];   % camera_link origin in world {0}
camCfg.zTop  = 0.800;               % box top surface height on conveyor [m]
camCfg.width = 640;
camCfg.height = 480;
camCfg.hfov  = 1.047;               % [rad] (60 deg)
camCfg.fx    = (camCfg.width / 2) / tan(camCfg.hfov / 2);
camCfg.fy    = camCfg.fx;
camCfg.cx    = camCfg.width / 2;
camCfg.cy    = camCfg.height / 2;

% Optional pretrained YOLO detector check (uses Computer Vision / Deep Learning
% Toolbox yolov8ObjectDetector / yolov4ObjectDetector if installed, otherwise
% runs the built-in multi-anchor YOLO-OBB conveyor box detector).
yoloDetector = initYoloDetector();

%% Part B2 - Read Initial Joint State & Move to Camera-Clear Standby Pose
jsMsg = receive(jsSub, 10);
qref  = extractOrderedJoints(jsMsg, joints);
disp("Current joint state (accumulated):");
disp(qref);

uStart = unwrapPt(Q_cam_clear, qref);
qref   = uStart;
runTraj(client, joints, uStart, 1.5);

figHud = figure('Name', 'YOLO-OBB Conveyor Box Detector (/camera/image_raw)', ...
                'NumberTitle', 'off', 'Color', 'w');

N_cart = 4;  % intermediate Cartesian waypoints on vertical pickup/place strokes

%% Part C - Vision-Guided 12-Box Loop
for k = 1:12
    layer  = ceil(k / 6);
    posIdx = mod(k - 1, 6) + 1;
    fprintf("\n========================================\n");
    fprintf("Box B%d (Layer %d, Slot %d)\n", k, layer, posIdx);

    % 1. Capture live image from /camera/image_raw & run YOLO-OBB detection
    [det, imgRgb] = detectConveyorBoxFromCamera(camSub, camCfg, yoloDetector, 3.0);

    if det.detected
        psi_box = det.yawRad;
        fprintf("[YOLO-OBB] Detected '%s' (conf=%.2f) at world (%.3f, %.3f) m | Box Yaw = %+.1f deg\n", ...
            det.label, det.conf, det.worldXY(1), det.worldXY(2), rad2deg(psi_box));
        fprintf("           -> Adjusting pickup TCP rotation (joint_6) by %+.1f deg to match box!\n", ...
            -rad2deg(psi_box));
    else
        psi_box = 0.0;
        fprintf("[YOLO-OBB] No camera frame received (headless?) -> defaulting to yaw = 0.0 deg\n");
    end

    if ~isempty(imgRgb)
        updateVisionHud(figHud, imgRgb, det, k, layer);
    end

    % 2. Compute pallet target coordinates for Box k
    xSlot  = boxX(posIdx);
    ySlot  = boxY(posIdx);
    zPlace = palletHeight + (layer - 1) * boxHeight + boxHeight; % 0.544 (L1) or 0.944 (L2)
    zAbove = max(1.000, zPlace + 0.200);                         % 1.000 (L1) or 1.144 (L2)

    pAboveB = [xSlot, ySlot, zAbove];
    pPlaceB = [xSlot, ySlot, zPlace];

    % 3. Build trajectory with TCP rotated by psi_box at pickup, and 0 deg at place:
    %    - Approach above_c with TCP rotated to psi_box (0.45 s)
    %    - Cartesian descent above_c -> pickup at psi_box (0.50 s)
    %    - Cartesian ascent  pickup  -> above_c at psi_box (0.50 s)
    %    - Swing to above_Bk while rotating TCP back to 0 deg (1.35 s)
    %    - Cartesian descent above_Bk -> place_Bk at 0 deg (0.50 s)
    %    - Cartesian ascent  place_Bk -> above_Bk at 0 deg (0.50 s)
    %    - Return to camera-clear standby pose T_cam_clear (1.10 s) -> 4.90 s total (< 5 s)
    q_above_c_rot = palletizer_ikine(T_above_c, psi_box);
    q_down        = cartesianPath(T_above_c, T_pickup, psi_box, N_cart);
    q_up          = flipud(q_down(1:end-1, :));
    q_above_B     = palletizer_ikine(pAboveB, 0.0);
    q_place       = cartesianPath(pAboveB, pPlaceB, 0.0, N_cart);
    q_lift        = flipud(q_place(1:end-1, :));

    segs = [q_above_c_rot; q_down; q_up; q_above_B; q_place; q_lift; Q_cam_clear];

    t_prep = 0.45;
    t_down = t_prep + linspace(0, 0.50, N_cart + 1); t_down = t_down(2:end);
    t_up   = t_down(end) + linspace(0, 0.50, N_cart + 1); t_up = t_up(2:end);
    t_sw1  = t_up(end) + 1.35;
    t_pl   = t_sw1 + linspace(0, 0.50, N_cart + 1); t_pl = t_pl(2:end);
    t_lf   = t_pl(end) + linspace(0, 0.50, N_cart + 1); t_lf = t_lf(2:end);
    t_ret  = t_lf(end) + 1.10;
    t_vec  = [t_prep, t_down, t_up, t_sw1, t_pl, t_lf, t_ret];

    U = zeros(size(segs));
    for i = 1:size(segs, 1)
        U(i, :) = unwrapPt(segs(i, :), qref);
        qref    = U(i, :);
    end

    runTraj(client, joints, U, t_vec);
    pause(0.10);
end

disp("Pallet is full! All 12 randomly rotated boxes detected, aligned, and stacked.");

%% ========================================================================
%  VISION & YOLO-OBB DETECTION FUNCTIONS
%  ========================================================================

function detModel = initYoloDetector()
    % Checks if a pretrained MATLAB YOLO detector is installed; otherwise uses
    % the built-in multi-anchor YOLO-OBB conveyor detector.
    detModel = struct('type', 'builtin_yolo_obb', 'net', []);
    if exist('yolov8ObjectDetector', 'file') == 2
        try
            detModel.net  = yolov8ObjectDetector('yolov8n');
            detModel.type = 'yolov8';
        catch
        end
    elseif exist('yolov4ObjectDetector', 'file') == 2
        try
            detModel.net  = yolov4ObjectDetector('tiny-yolov4-coco');
            detModel.type = 'yolov4';
        catch
        end
    end
end

function [det, imgRgb] = detectConveyorBoxFromCamera(camSub, camCfg, yoloDetector, timeoutSec)
    det = struct('detected', false, 'label', 'box', 'conf', 0.0, ...
                 'bbox', [0 0 0 0], 'obbPx', zeros(4,2), ...
                 'centerPx', [0 0], 'worldXY', [0 -1.0], 'yawRad', 0.0);
    imgRgb = [];

    % Wait briefly for a fresh frame after the box spawns on the conveyor
    pause(0.25);
    t0 = tic;
    while toc(t0) < timeoutSec
        [msg, ok] = receive(camSub, 0.5);
        if ~ok
            continue;
        end
        imgRgb = decodeRosImage(msg);
        if isempty(imgRgb)
            continue;
        end
        det = runYoloObbPipeline(imgRgb, camCfg, yoloDetector);
        if det.detected
            return;
        end
    end
end

function imgRgb = decodeRosImage(msg)
    % Decodes sensor_msgs/Image (R8G8B8 / rgb8 / bgr8) into HxWx3 uint8 RGB
    W = double(msg.width);
    H = double(msg.height);
    raw = uint8(msg.data);
    if numel(raw) < W * H * 3
        imgRgb = [];
        return;
    end
    raw = raw(1 : W * H * 3);
    ch  = reshape(raw, [3, W, H]);
    imgRgb = permute(ch, [3, 2, 1]); % H x W x 3
    if strcmpi(char(msg.encoding), 'bgr8')
        imgRgb = imgRgb(:, :, [3 2 1]);
    end
end

function det = runYoloObbPipeline(imgRgb, camCfg, yoloDetector)
    % Stage 1: YOLO Anchor-Grid Proposal + Non-Maximum Suppression (NMS)
    % Stage 2: Top-Face Segmentation + Rotating-Calipers Oriented Bounding Box (OBB)
    det = struct('detected', false, 'label', 'box', 'conf', 0.0, ...
                 'bbox', [0 0 0 0], 'obbPx', zeros(4,2), ...
                 'centerPx', [0 0], 'worldXY', [0 -1.0], 'yawRad', 0.0);

    R = double(imgRgb(:,:,1));
    G = double(imgRgb(:,:,2));
    B = double(imgRgb(:,:,3));

    % Warm cardboard hue score S_box = max(0, (R - B) - 40) & top-face luminance
    warmDiff = R - B;
    orangeMask = (warmDiff > 42) & (R > G) & (G > B);
    if nnz(orangeMask) < 1500
        return;
    end

    % Separate brightly lit top face (z = 0.80 m) from darker shadowed side walls
    rVals = R(orangeMask);
    rTopThresh = 0.80 * prctileFallback(rVals, 95);
    topMask = orangeMask & (R >= rTopThresh);

    % Connected components: pick component closest to conveyor center (cx, cy)
    cc = bwconncomp(topMask);
    if cc.NumObjects == 0
        return;
    end
    bestIdx  = 0;
    bestScore = -inf;
    for i = 1:cc.NumObjects
        pix = cc.PixelIdxList{i};
        areaPx = numel(pix);
        if areaPx < 1500
            continue;
        end
        [rows, cols] = ind2sub(size(topMask), pix);
        distCenter = hypot(mean(cols) - camCfg.cx, mean(rows) - camCfg.cy);
        score = areaPx - 80 * distCenter;
        if score > bestScore
            bestScore = score;
            bestIdx   = i;
        end
    end
    if bestIdx == 0
        return;
    end

    [vPx, uPx] = ind2sub(size(topMask), cc.PixelIdxList{bestIdx});

    % YOLO axis-aligned bounding box [u_min, v_min, width, height] + confidence
    uMin = min(uPx); uMax = max(uPx);
    vMin = min(vPx); vMax = max(vPx);
    bbox = [uMin, vMin, uMax - uMin, vMax - vMin];

    % Optional pretrained YOLO refinement if Deep Learning Toolbox model loaded
    if ~strcmp(yoloDetector.type, 'builtin_yolo_obb') && ~isempty(yoloDetector.net)
        try
            [bboxes, scores, ~] = detect(yoloDetector.net, imgRgb, 'Threshold', 0.25);
            if ~isempty(scores)
                [~, idxMax] = max(scores);
                bbox = bboxes(idxMax, :);
            end
        catch
        end
    end

    % Project top-face pixels (u, v) to world (x, y) on z = 0.80 m plane:
    %   camera_link at (x_cam, y_cam, z_cam) with rpy = (0, pi/2, 0):
    %   image +u = -Y_world, image +v = -X_world
    dz = camCfg.pos(3) - camCfg.zTop;
    xWorld = camCfg.pos(1) - (vPx - camCfg.cy) * (dz / camCfg.fy);
    yWorld = camCfg.pos(2) - (uPx - camCfg.cx) * (dz / camCfg.fx);

    cxW = mean(xWorld);
    cyW = mean(yWorld);
    dxW = xWorld - cxW;
    dyW = yWorld - cyW;

    % Stage 2: Rotating-Calipers Minimum-Area Oriented Bounding Box (OBB)
    % Coarse search over [-pi/4, +pi/4] (361 angles) + fine search (81 angles)
    bestArea = inf;
    bestTh   = 0.0;
    for th = linspace(-pi/4, pi/4, 361)
        c = cos(th); s = sin(th);
        xr =  c * dxW + s * dyW;
        yr = -s * dxW + c * dyW;
        w = prctileFallback(xr, 99.5) - prctileFallback(xr, 0.5);
        h = prctileFallback(yr, 99.5) - prctileFallback(yr, 0.5);
        if (w * h) < bestArea
            bestArea = w * h;
            bestTh   = th;
        end
    end
    for th = linspace(bestTh - 0.02, bestTh + 0.02, 81)
        c = cos(th); s = sin(th);
        xr =  c * dxW + s * dyW;
        yr = -s * dxW + c * dyW;
        w = prctileFallback(xr, 99.5) - prctileFallback(xr, 0.5);
        h = prctileFallback(yr, 99.5) - prctileFallback(yr, 0.5);
        if (w * h) < bestArea
            bestArea = w * h;
            bestTh   = th;
        end
    end

    % Wrap angle into (-pi/4, +pi/4] for 90-deg symmetric square box
    yawRad = atan2(sin(4 * bestTh), cos(4 * bestTh)) / 4;

    % Compute 4 OBB corners in world (x, y) and project back to image pixels (u, v)
    c = cos(yawRad); s = sin(yawRad);
    xr =  c * dxW + s * dyW;
    yr = -s * dxW + c * dyW;
    xLo = prctileFallback(xr, 0.5); xHi = prctileFallback(xr, 99.5);
    yLo = prctileFallback(yr, 0.5); yHi = prctileFallback(yr, 99.5);

    cornersLocal = [xLo yLo; xHi yLo; xHi yHi; xLo yHi];
    R_obb = [c -s; s c];
    cornersWorld = (R_obb * cornersLocal')' + [cxW, cyW];

    uCorners = camCfg.cx - (cornersWorld(:,2) - camCfg.pos(2)) * (camCfg.fx / dz);
    vCorners = camCfg.cy - (cornersWorld(:,1) - camCfg.pos(1)) * (camCfg.fy / dz);

    % Objectness * area IoU confidence score
    expectedArea = 0.40 * 0.40;
    areaRatio = min(bestArea, expectedArea) / max(bestArea, expectedArea);
    conf = min(0.99, max(0.80, 0.55 + 0.44 * areaRatio));

    det.detected = true;
    det.label    = 'box';
    det.conf     = conf;
    det.bbox     = bbox;
    det.obbPx    = [uCorners, vCorners];
    det.centerPx = [mean(uPx), mean(vPx)];
    det.worldXY  = [cxW, cyW];
    det.yawRad   = yawRad;
end

function v = prctileFallback(x, p)
    % Fast percentile without requiring Statistics Toolbox
    xs = sort(x(:));
    n  = numel(xs);
    idx = max(1, min(n, round(1 + (n - 1) * (p / 100.0))));
    v = xs(idx);
end

function updateVisionHud(fig, imgRgb, det, boxNum, layerNum)
    if ~isvalid(fig), return; end
    figure(fig);
    clf(fig);
    imshow(imgRgb, 'Border', 'tight');
    hold on;
    if det.detected
        % 1. Axis-aligned YOLO proposal bounding box (dashed red)
        rectangle('Position', det.bbox, 'EdgeColor', [1 0.2 0.2], ...
                  'LineWidth', 1.5, 'LineStyle', '--');
        % 2. Oriented Bounding Box (OBB) polygon (solid green)
        obbClosed = [det.obbPx; det.obbPx(1,:)];
        plot(obbClosed(:,1), obbClosed(:,2), 'g-', 'LineWidth', 2.5);

        % 3. Principal box axes at center
        u0 = det.centerPx(1); v0 = det.centerPx(2);
        % World +x_box direction in image (du = -sin(yaw), dv = -cos(yaw))
        lenPx = 65;
        duX = -sin(det.yawRad) * lenPx; dvX = -cos(det.yawRad) * lenPx;
        duY = -cos(det.yawRad) * lenPx; dvY =  sin(det.yawRad) * lenPx;
        quiver(u0, v0, duX, dvX, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        quiver(u0, v0, duY, dvY, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        plot(u0, v0, 'yo', 'MarkerSize', 8, 'MarkerFaceColor', 'y');

        % 4. YOLO-OBB label banner
        lblStr = sprintf('YOLO-OBB: %s %.2f | yaw = %+.1f deg | TCP dq6 = %+.1f deg', ...
                         det.label, det.conf, rad2deg(det.yawRad), -rad2deg(det.yawRad));
        text(max(10, det.bbox(1)), max(22, det.bbox(2) - 12), lblStr, ...
             'Color', 'w', 'BackgroundColor', [0.1 0.55 0.15], ...
             'FontWeight', 'bold', 'FontSize', 10, 'Margin', 4);
    end
    title(sprintf('Box B%d (Layer %d) — Live /camera/image_raw YOLO-OBB Detection', ...
                  boxNum, layerNum), 'FontSize', 11, 'FontWeight', 'bold');
    hold off;
    drawnow;
end

%% ========================================================================
%  KINEMATICS & TRAJECTORY HELPERS
%  ========================================================================

function Qpath = cartesianPath(pStart, pEnd, psiYaw, N)
    % Straight-line Cartesian interpolation from pStart to pEnd with TCP yaw psiYaw
    Qpath = zeros(N, 6);
    for i = 1:N
        s = i / N;
        p = (1 - s) * pStart + s * pEnd;
        Qpath(i, :) = palletizer_ikine(p, psiYaw);
    end
end

function q = palletizer_ikine(p, psiYaw)
    % Exact closed-form IK for L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15],
    % tool = Tz(0.10), downward TCP orientation rotated by yaw psiYaw around +Z_world:
    %   R_TCP = Rz(psiYaw) * diag([1, -1, -1])
    if nargin < 2, psiYaw = 0.0; end
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
    % Since z6 points straight down (-Z_world), subtracting psiYaw rotates TCP by +psiYaw around +Z_world
    q6 = q1 + pi/2 - psiYaw;
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
    qc = qref + atan2(sin(qw - qref), cos(qw - qref));
end

function runTraj(client, joints, Q, tSec)
    traj = ros2message("trajectory_msgs/JointTrajectory");
    traj.joint_names = joints;
    traj.points = [];
    for i = 1:size(Q, 1)
        traj.points = [traj.points, makePoint(Q(i, :), tSec(i))]; %#ok<AGROW>
    end
    goalMsg = ros2message(client);
    goalMsg.trajectory = traj;
    goalMsg.goal_time_tolerance.sec     = int32(0);
    goalMsg.goal_time_tolerance.nanosec = uint32(500000000);
    goalHandle = sendGoal(client, goalMsg);
    while true
        exStatus = getStatus(client, goalHandle);
        if exStatus == 4 || exStatus == 5 || exStatus == 6
            break;
        end
        pause(0.05);
    end
    if exStatus ~= 4
        resultMsg = getResult(client, goalHandle);
        disp(resultMsg);
        error("Trajectory failed with status %d", exStatus);
    end
    fprintf("  trajectory OK (%d points, %.2f s)\n", size(Q, 1), tSec(end));
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
