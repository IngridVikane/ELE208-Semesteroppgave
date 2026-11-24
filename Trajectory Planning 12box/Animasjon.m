%% Denne filen produserer en video av baneplanleggingen. 

%    Copyright:     HVL
%    Project:	    ELE208 - Robotmanipulatorer
%    Author:        
%    Date created:  Høst 2026
%    Input: 'Workspace.m', 'pallet_cycles.mat'       
%           
%    Output: 'Palletizing_Cycles.mp4'       
%---------------------------------------------------------------------%
%%
% Note: Laget kode for animasjon av robotarmen i MatLab, vha. ChatGPT. Det var
% ønskelig å separere
% animeringen fra 'InverseKinematics12boxes.m', og heller lagre banene fra
% scriptet i 'pallet_cycles.mat' som kan brukes av flere programmer. Da
% slipper vi at inverskinematikken må regnes ut for 12 sykluser hver gang
% vi skal teste noe. 

close all; clc;

run('Workspace.m');                 % Tegner arbeidsområdet og lager robotmodellen
load('pallet_cycles.mat');          % Gir variabelen 'cycles'

boxesToShow = 1:numel(cycles);      % Hvilke esker som animeres, f.eks. 6 eller 1:12
saveVideo   = false;                % true = lagre MP4
fps         = 25;                   % Bilder per sekund

fig = findobj('Type', 'figure', 'Name', '6DOF Palletizing Workspace');
fig = fig(1);
figure(fig);

if saveVideo
    video = VideoWriter('Palletizing_Cycles.mp4', 'MPEG-4');
    video.FrameRate = fps;
    open(video);
end

%% Animer
for k = boxesToShow
    cyc = cycles(k);
    fprintf('Animerer syklus %d (%.2f s)\n', k, cyc.t(end));  % Viser hvor lang tid hver syklus tar 

    % Sampler banen med fast tid mellom hvert bilde, slik at videoen
    % får riktig tempo (sakte i start/stopp, raskt på midten)
    tVideo = 0 : 1/fps : cyc.t(end);
    qVideo = interp1(cyc.t, cyc.q, tVideo);

    for i = 1:numel(tVideo)
        Palletizing.animate(qVideo(i,:));
        drawnow;

        if saveVideo
            writeVideo(video, getframe(fig));
        end
    end
end

if saveVideo
    close(video);
    disp('Video lagret: Palletizing_Cycles.mp4');
end