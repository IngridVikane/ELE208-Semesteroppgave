run('Palletizing_Robotarm.m');

%% robot model construction

% 1. Define the physical robot and link dynamics
% Dynamic parameters
Palletizing.links(2).m = 5.0;               % Link Mass
Palletizing.links(2).r = [-(L2/2) 0 0];     % Center of Mass positions (CoM)







