%% Denne filen regner ut differensialkinematikken.  

% % 
%    Copyright:     HVL
%    Project:	    ELE208 - Robotmanipulatorer
%    Author:        Erlend Østbø
%    Coauthor:      Ingrid Vikane   (Knyttet programmet til baneplanleggingen, endret jtraj ved opplukk og nedsetting til cartesian space)
%    Date created:  Høst 2026
%    Input: 'Workspace.m' og 'pallet_cycles.mat'
%           
%    Output: 'Joint accelerations - One full palletizing cycle' + qd, qdd
%---------------------------------------------------------------------%
%%
% Note: 
%

%% Differential Kinematics - First position complete move sequence
clc;

run('Workspace.m');
load('pallet_cycles.mat'); 

%% Henter banen fra baneplanleggingen
boxIdx = 1;                             % Hvilken syklus som analyseres
cyc = cycles(boxIdx);

% Knytter baneplanleggingsparameterene opp til parameterene i dette
% programmet
t             = cyc.t;
qPath         = cyc.q;
qVelocity     = cyc.qd;
qAcceleration = cyc.qdd;


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

fprintf('Total movement time: %.2f s\n', t(end));

%% graf 1 leddpositioner

figure;
plot(t,rad2deg(qPath),'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Joint angle [deg]');
title('Joint positions - One full palletizing cycle');
legend('J1','J2','J3','J4','J5','J6');
grid on;

%% graf 2 leddhastigheter over tid

figure;
hold on;

% Joint velocities
plot(t,rad2deg(qVelocity),'LineWidth',1.5);

%-------------------------------------------------------------------------%

%Oppdeling av graf laget med ai

% Phase boundaries
xline(0.9,'--w','LineWidth',1.2);
xline(3.5,'--w','LineWidth',1.2);

text(0.5,145,'Pickup', ...
    'HorizontalAlignment','center','Color','w');

text(2.25,145,'Palletizing', ...
    'HorizontalAlignment','center','Color','w');

text(4.25,145,'Return', ...
    'HorizontalAlignment','center','Color','w');

xlabel('Time [s]');
ylabel('Joint velocity [deg/s]');
title('Joint velocities - One full palletizing cycle');

legend('J1','J2','J3','J4','J5','J6', ...
    'Location','northeast');

xlim([0 t(end)]);

grid on;
hold off;
%-------------------------------------------------------------------------%

%% graf 3 TCP speed 

figure;
plot(t,tcpSpeed,'LineWidth',1.5);
xlabel('Time [s]');
ylabel('TCP speed [m/s]');
title('TCP speed - One full palletizing cycle');
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
title('Joint accelerations - One full palletizing cycle');

legend('J1','J2','J3','J4','J5','J6');
grid on;
hold off