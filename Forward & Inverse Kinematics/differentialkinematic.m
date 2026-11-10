%% Differential Kinematics - First position complete move sequence

clear;
clc;

run('Palletizing_Robotarm.m');

%% Positions

R = trotx(pi);

% box pickup position
T_start  = transl(0,-1.0,1.0)*R;
T_pickup = transl(0,-1.0,0.8)*R;

% Pallet position 1
T_above_place = transl(1.0,0.4,1.0)*R;
T_place = transl(1.0,0.4,pallet.height+0.4)*R;

%% Inverse Kinematics

q0 = deg2rad([90 -30 60 0 30 0]);

qStart = Palletizing.ikcon(T_start,q0);
qPickup = Palletizing.ikcon(T_pickup,qStart);

qAbovePlace = Palletizing.ikcon(T_above_place,qStart);
qPlace = Palletizing.ikcon(T_place,qAbovePlace);

%% steps

N = 100;

% total movement time
t1 = 0.5;   % Start -> Pickup
t2 = 0.5;   % Pickup -> Start (straight up)
t3 = 1.5;   % Start -> Above place
t4 = 0.5;   % Above place -> Place
t5 = 0.5;   % Place -> Above place
t6 = 1.5;   % Above place -> Start

%% trajectories for a full sequence move

%% Generate trajectories - position, speed, acceleration

[q1,qd1,qdd1] = jtraj(qStart,qPickup,linspace(0,t1,N));
[q2,qd2,qdd2] = jtraj(qPickup,qStart,linspace(0,t2,N));
[q3,qd3,qdd3] = jtraj(qStart,qAbovePlace,linspace(0,t3,N));
[q4,qd4,qdd4] = jtraj(qAbovePlace,qPlace,linspace(0,t4,N));
[q5,qd5,qdd5] = jtraj(qPlace,qAbovePlace,linspace(0,t5,N));
[q6,qd6,qdd6] = jtraj(qAbovePlace,qStart,linspace(0,t6,N));

%% combine trajectoriees

qPath = [q1; q2(2:end,:); q3(2:end,:); ...
         q4(2:end,:); q5(2:end,:); q6(2:end,:)];

qVelocity = [qd1; qd2(2:end,:); qd3(2:end,:); ...
             qd4(2:end,:); qd5(2:end,:); qd6(2:end,:)];

qAcceleration = [
    qdd1;
    qdd2(2:end,:);
    qdd3(2:end,:);
    qdd4(2:end,:);
    qdd5(2:end,:);
    qdd6(2:end,:)
    ];

t = [linspace(0,t1,N), ...
     t1+linspace(0,t2,N-1), ...
     t1+t2+linspace(0,t3,N-1), ...
     t1+t2+t3+linspace(0,t4,N-1), ...
     t1+t2+t3+t4+linspace(0,t5,N-1), ...
     t1+t2+t3+t4+t5+linspace(0,t6,N-1)];

%% differential kinematic

tcpVelocity = zeros(size(qPath,1),6);

for i = 1:size(qPath,1)

    J = Palletizing.jacob0(qPath(i,:));

    tcpVelocity(i,:) = (J*qVelocity(i,:)')';

end

tcpSpeed = vecnorm(tcpVelocity(:,1:3),2,2);

%% maks hastighet

maxJointVelocity = max(abs(rad2deg(qVelocity)),[],1);

disp('Maximum joint velocities [deg/s]:');
disp(maxJointVelocity);

fprintf('Maximum TCP speed: %.3f m/s\n',max(tcpSpeed));

fprintf('Total movement time: %.2f s\n',sum([t1 t2 t3 t4 t5 t6]));

%% graf 1 leddpositioner

figure;
plot(t,rad2deg(qPath),'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Joint angle [deg]');
title('Joint positions - Full palletizing cycle');
legend('J1','J2','J3','J4','J5','J6');
grid on;

%% graf 2 leddhastigheter over tid

figure;
hold on;

% Joint velocities
plot(t,rad2deg(qVelocity),'LineWidth',1.5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Oppdeling av graf laget med ai

% Phase boundaries
xline(1.0,'--w','LineWidth',1.2);
xline(3.5,'--w','LineWidth',1.2);

text(0.5,145,'Pickup', ...
    'HorizontalAlignment','center','Color','w');

text(2.25,145,'Palletizing', ...
    'HorizontalAlignment','center','Color','w');

text(4.25,145,'Return', ...
    'HorizontalAlignment','center','Color','w');

xlabel('Time [s]');
ylabel('Joint velocity [deg/s]');
title('Joint velocities - Full palletizing cycle');

legend('J1','J2','J3','J4','J5','J6', ...
    'Location','northeast');

xlim([0 5]);
ylim([-150 160]);

grid on;
hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% graf 3 TCP speed 

figure;
plot(t,tcpSpeed,'LineWidth',1.5);
xlabel('Time [s]');
ylabel('TCP speed [m/s]');
title('TCP speed - Full palletizing cycle');
grid on;


%% graf 4 ledd akselerasjoner 

figure;
hold on;

plot(t,rad2deg(qAcceleration),'LineWidth',1.5);

% Phase boundaries
xline(1.0,'--w','LineWidth',1.2);
xline(3.5,'--w','LineWidth',1.2);

xlabel('Time [s]');
ylabel('Joint acceleration [deg/s^2]');
title('Joint accelerations - Full palletizing cycle');

legend('J1','J2','J3','J4','J5','J6');
grid on;
hold off