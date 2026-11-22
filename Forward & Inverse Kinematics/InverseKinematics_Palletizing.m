clc; 
clear; 

% Kjører Palletizing Robotarm & Workspace
run('Palletizing_Robotarm.m');
run('Workspace.m');

%% Trajectories/Baneplanlegging
% Over transportbåndet (T_above_c) og plukker opp fra transportbåndet (T_pickup)
T_above_c = transl(0, -1.0, 1.0)*trotx(pi);
T_pickup = transl(0, -1.0, 0.8) * trotx(pi);                         

% Over forhåndsdefinerte eskeposisjoner
T_above_B1 = transl(0.6, -0.4, 1.0)*trotx(pi);
T_above_B2 = transl(0.6, 0.0, 1.0)*trotx(pi);
T_above_B3 = transl(0.6, 0.4, 1.0)*trotx(pi);
T_above_B4 = transl(1.0, -0.4, 1.0)*trotx(pi);
T_above_B5 = transl(1.0, 0.0, 1.0)*trotx(pi);
T_above_B6 = transl(1.0, 0.4, 1.0)*trotx(pi);

% Plasserer esker ned i forhåndsdefinerte posisjoner
T_B1 = transl(0.6, -0.4, pallet.height + box.size) * trotx(pi);
T_B2 = transl(0.6, 0.0, pallet.height + box.size) * trotx(pi);
T_B3 = transl(0.6, 0.4, pallet.height + box.size) * trotx(pi);
T_B4 = transl(1.0, -0.4, pallet.height + box.size) * trotx(pi);
T_B5 = transl(1.0, 0.0, pallet.height + box.size) * trotx(pi);
T_B6 = transl(1.0, 0.4, pallet.height + box.size) * trotx(pi);

%% Inverse Kinematics/Inverskinematikk vha ikcon()
q0 = deg2rad([20 20 20 20 20 20]);      % Startkonfigurasjon

% For transportbåndet
q_above_c = Palletizing.ikcon(T_above_c, q0);
q_pickup = Palletizing.ikcon(T_pickup, q_above_c);

% For plasseringen av boksene
q_above_B1 = Palletizing.ikcon(T_above_B1, q_above_c)
q_B1 = Palletizing.ikcon(T_B1, q_above_B1);

q_above_B2 = Palletizing.ikcon(T_above_B2, q_above_c)
q_B2 = Palletizing.ikcon(T_B2, q_above_B2);

q_above_B3 = Palletizing.ikcon(T_above_B3, q_above_c)
q_B3 = Palletizing.ikcon(T_B3, q_above_B3);

q_above_B4 = Palletizing.ikcon(T_above_B4, q_above_c)
q_B4 = Palletizing.ikcon(T_B4, q_above_B4);

q_above_B5 = Palletizing.ikcon(T_above_B5, q_above_c)
q_B5 = Palletizing.ikcon(T_B5, q_above_B5);

q_above_B6 = Palletizing.ikcon(T_above_B6, q_above_c)
q_B6 = Palletizing.ikcon(T_B6, q_above_B6);

%% Visualisering
fig = findobj('Type', 'figure', ...
    'Name', '6DOF Pallsettingsrobot i Arbeidsområdet');

Palletizing.plot(q_above_c);
title('Over transportbåndet');

Palletizing.plot(q_B1);
title('Klar til å plassere eske B1');

% Beregner TCP-posisjonen (vha. fk) for å dobbelsjekke
T_TCP_A = Palletizing.fkine(q_above_c);
TCP_A = transl(T_TCP_A)

T_TCP_B = Palletizing.fkine(q_B1);
TCP_B = transl(T_TCP_B)

%% Joint-space Bane: Mellom transportbåndet og pallen (jtraj)
% B1
T_move = 3;
N = 100;
t = linspace(0, T_move, N);

[q, qd, qdd] = jtraj(q_above_c, q_above_B1, t);

figure
Palletizing.plot(q);

% Beregner TCP posisjonen for hver
p = zeros(N, 3);

for i = 1:N
    T_actual = Palletizing.fkine(q(i,:));
    p(i,:) = transl(T_actual);
end

% Tegner TCP banen
figure
plot3(p(:,1), p(:,2), p(:,3), 'LineWidth', 2)
grid on
axis equal

xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
title('TCP bane - Joint-space trajectory')

hold on
plot3(p(1,1), p(1,2), p(1,3), 'go');
plot3(p(end,1), p(end,2), p(end,3), 'ro');

legend('TCP Bane', 'Start', 'Slutt');

q_return = flipud(q);

%% Kartesisk Bane - Opp-plukking
N = 100;
s = linspace(0, 1, N);

% Start og sluttposisjoner (fikk litt problemer med matrise multiplikasjon)
pA = reshape(transl(T_pickup), 1, 3);
pB = reshape(transl(T_above_c), 1, 3);

% Lineær interpolering av TCP-posisjon
p = (1-s')*pA + s'*pB;

% Ønsket TCP-orientering
R = trotx(pi);

% Sett opp ønskede poses
T = cell(N,1);

for i = 1:N
    T{i} = transl(p(i,:)) * R;
end

% Invers kinematikk
q_pick = zeros(N,6);
q_pick(1,:) = q_pickup;

for i = 2:N
    q_pick(i,:) = Palletizing.ikcon(T{i}, q_pick(i-1,:));
end

figure
Palletizing.plot(q_pick);
title('Cartesian Trajectory - Pickup');

%% Kartesisk bane - Plassere B1

N = 100;
s = linspace(0, 1, N);

% Start og sluttposisjoner (fikk litt problemer med matrise multiplikasjon)
pA = reshape(transl(T_above_B1), 1, 3);
pB = reshape(transl(T_B1), 1, 3);

% Lineær interpolering av TCP-posisjon
p = (1-s')*pA + s'*pB;

% Ønsket TCP-orientering
R = trotx(pi);

% Sett opp ønskede poses
T = cell(N,1);

for i = 1:N
    T{i} = transl(p(i,:)) * R;
end

% Inverse kinematics/Inverskinematikk
q_place = zeros(N,6);
q_place(1,:) = q_above_B1;

for i = 2:N
    q_place(i,:) = Palletizing.ikcon(T{i}, q_place(i-1,:));
end

q_lift = flipud(q_place);   % Reverserer matrisen, altså bevegelsen

% Animere robot
figure
Palletizing.plot(q_place);
title('Cartesian Trajectory - Placement B1');

%% Kombinerer banene (gir én syklus)

q_total = [
    q_pick;
    q(2:end,:);
    q_place(2:end,:)
    q_lift(2:end,:);
    q_return(2:end,:) 
    ];

%% Kontroller hastigheter og akselerasjon

qd_max = max(abs(qd), [], 1);
qdd_max = max(abs(qdd), [], 1);

disp('Maksimal leddhastighet [rad/s]:')
disp(qd_max)

disp('Maksimal leddakselerasjon [rad/s^2]:')
disp(qdd_max)


%% Lagre robotanimasjon som MP4
% https://se.mathworks.com/help/matlab/ref/videowriter.html
% ChatGPT

% Open workspace figure
figure(fig);

% Set robot to initial configuration
Palletizing.plot(q_total(1,:));
drawnow;

% Create video file
video = VideoWriter('Palletizing_Trajectory.mp4', 'MPEG-4');
video.FrameRate = 20;

open(video);

% Record robot movement
for i = 1:size(q_total,1)

    % Update robot position
    Palletizing.animate(q_total(i,:));
    drawnow;

    % Capture frame
    frame = getframe(fig);

    % Write frame to video
    writeVideo(video, frame);

end

% Close and save video
close(video);

disp('Video saved successfully!');

