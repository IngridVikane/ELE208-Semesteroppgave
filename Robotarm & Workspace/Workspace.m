%% Denne filen produserer et arbeidsområde (Workspace) for vår 
%% 6DOF Palletizing Robotarm.

% % 
%    Copyright:     HVL
%    Project:	    ELE308 - Robotmanipulatorer
%    Author:        Ingrid Vikane
%    Date created:  Høst 2026
%    Input: 'Palletizing_Robotarm.m'       
%           
%    Output: '6DOF Palletizing Robot Workspace'       
%---------------------------------------------------------------------%
%%
% Note: Denne filen består av transformasjoner mellom robotens base (verdenskoordinatsystemet {0}) 
% og alle objektene i arbeidsområdet. I tillegg en transformasjon mellom
% esken og kamera. Disse
% brukes for å plassere alle objektene i arbeidsområdet. 
% I tillegg blir arbeidsområdet visualisert ved hjelp av patch. 

run('Palletizing_Robotarm.m');

%% Arbeidsområde/Workspace 

%% Arbeidsområde transformasjoner – Alle i forhold til robotbasen {0} (verdenskoordinatsystemet)

% Transformasjon fra robot {0} i forhold til transportbånd (Conveyor) {C}
conveyorPosition = [0, -1.25, 0];
conveyorRotation = 0;


T0C = transl(conveyorPosition) * trotz(conveyorRotation);


% Transformasjon fra robot {0} i forhold til pall {P}
palletPosition = [0.8, 0, 0];
palletRotation = pi/2;


T0P = transl(palletPosition) * trotz(palletRotation);


% Transformasjon fra kamera {K} relativt til transportbånd {C}
cameraPosition = [0, 0, 2.0];
cameraRotation = pi;            % Peker ned mot transportbåndet

TCK = transl(cameraPosition)*trotx(cameraRotation);

T0K = T0C * TCK;                % Transformasjon fra kamera {K} relativt til robotbasen {0} (siden det er montert ovenfor)

% Transformasjon fra eske {B} relativt til transportbånd {C}
boxPosition = [0, 0.25, 0.4];
boxRotation = 0;

TCB = transl(boxPosition)*trotz(boxRotation);

T0B = T0C * TCB;                % Transformasjon fra boks {B} relativt til robotbasis {0}

% Transformasjon fra kamera {K} relativt til boks {B} (For kamera som detekterer boks – ingen faktiske data)
x_detected = 0.10;              % Registrert x-posisjon
y_detected = 0.05;              % Registrert y-posisjon
z_detected = 0.40;              % Registrert z-posisjon

TKB = transl(x_detected, y_detected, z_detected);

T0B_K = T0K *  TKB;             % Transformasjon av posisjonen til den detekterte boksen fra kamera {K} i forhold til robotbasen {0}


% Arbeidsområde Dimensjoner
conveyor.length     = 0.6;      % Må få plass i en eske (400x400x400 mm) -> 10 cm klaring på hver side av esken
conveyor.width      = 1.0;      
conveyor.height     = 0.4;      % Maks. vertikal rekkevidde 1 m (0,4 m + 0,4 m (eske) = 0,8 m) -> 0,2 m klaring til å løfte esken opp

pallet.length       = 1.20;     % Standard europeisk pall
pallet.width        = 0.80;
pallet.height       = 0.144;

box.size            = 0.4;      % Eske (400x400x400 mm) -> 0,4 ​​m


camera.length       = 0.12;
camera.width        = 0.08;
camera.height       = 0.06;

camera.z            = 2.00;     % Høyde over bakken


%%-----------------------------------------------------------------------------------------------------------------------%%


% Ønsket TCP-startposisjon
T_start = transl(0,-1.0,1.0)*trotx(pi);

% Innledende gjetting for invers kinematikk
qSeed = deg2rad([-75 -20 -20 -30 90 0]);

%% Visualisering av robot
q0 = Palletizing.ikcon(T_start,qSeed);

figure('Name','6DOF Palletizing Workspace');

% Robot i arbeidsområdet
Palletizing.plot(q0, ...
    'workspace', [-1.8 1.8 -1.8 1.8 -0.1 1.8]);

% TCP-transformasjon
T0TCP_transformation = Palletizing.fkine(q0);

disp('TCP position [m]:');                  % For å styre posisjonen til TCP-en når vi beveger roboten
transl(T0TCP_transformation)


%% Visualisering av arbeidsområdet ved bruk av patch()
% Referanser: 
% https://se.mathworks.com/help/matlab/visualize/multifaceted-patches.html 
% + https://se.mathworks.com/help/matlab/ref/matlab.graphics.primitive.patch-properties.html 
% + ChatGPT (OpenAI) was used as a guide when implementing patch() 
% Prompt: https://chatgpt.com/share/6ad4f776-00b4-83ed-ac72-f81b8dbd6c09 

hold on;

% Flater for rektangulært 3D-objekt
F = [ ...
    1 2 3 4;   % Bottom
    5 6 7 8;   % Top
    1 2 6 5;   % Front
    2 3 7 6;   % Right
    3 4 8 7;   % Back
    4 1 5 8];  % Left

% Transportbånd

l = conveyor.length;
w = conveyor.width;
h = conveyor.height;

% Vertices for conveyor
V = [ ...
    -l/2, -w/2, 0;
     l/2, -w/2, 0;
     l/2,  w/2, 0;
    -l/2,  w/2, 0;
    -l/2, -w/2, h;
     l/2, -w/2, h;
     l/2,  w/2, h;
    -l/2,  w/2, h];

% Konverter til homogene koordinater
V_hc = [V, ones(size(V,1),1)]';

% Transformer fra {C} til {0}     - Implementer korrekt transformasjon
V_world_hc = T0C * V_hc;

% Konverter tilbake til kartesiske koordinater
V_world = V_world_hc(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0.4 0.7 0.9], ...
      'FaceAlpha', 0.7, ...
      'EdgeColor', 'k');

% Pall

l = pallet.length;
w = pallet.width;
h = pallet.height;

V = [
    -l/2 -w/2 0;
     l/2 -w/2 0;
     l/2  w/2 0;
    -l/2  w/2 0;
    -l/2 -w/2 h;
     l/2 -w/2 h;
     l/2  w/2 h;
    -l/2  w/2 h
];


% Konverter til homogene koordinater
V_hp = [V, ones(size(V,1),1)]';

% Transformer fra {P} til {0}
V_world_hp = T0P * V_hp;

% Konverter tilbake til kartesiske koordinater
V_world = V_world_hp(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0.6 0.4 0.2], ...
      'FaceAlpha', 0.8, ...
      'EdgeColor', 'k');


% Eske på transportbånd

l = box.size;
w = box.size;
h = box.size;

V = [
    -l/2 -w/2 0;
     l/2 -w/2 0;
     l/2  w/2 0;
    -l/2  w/2 0;
    -l/2 -w/2 h;
     l/2 -w/2 h;
     l/2  w/2 h;
    -l/2  w/2 h
];

% Konverter til homogene koordinater
V_hb = [V, ones(size(V,1),1)]';

% Transformer fra {B} til {0} - Implementer korrekt transformasjon
V_world_hb = T0B * V_hb;

% Konverter tilbake til kartesiske koordinater
V_world = V_world_hb(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0.8 0.5 0.2], ...
      'FaceAlpha', 0.9, ...
      'Edgecolor', 'k');


% Kamera

l = camera.length;
w = camera.width;
h = camera.height;

V = [
    -l/2 -w/2 0;
     l/2 -w/2 0;
     l/2  w/2 0;
    -l/2  w/2 0;
    -l/2 -w/2 h;
     l/2 -w/2 h;
     l/2  w/2 h;
    -l/2  w/2 h
];

% Konverter til homogene koordinater
V_hk = [V, ones(size(V,1),1)]';

% Transformer fra {K} til {0} - Implementer korrekt transformasjon
V_world_hk = T0K * V_hk;

% Konverter tilbake til kartesiske koordinater
V_world = V_world_hk(1:3,:)';

patch('Vertices', V_world, ...
      'Faces', F, ...
      'FaceColor', [0 0 0.15], ...
      'FaceAlpha', 1.0, ...
      'Edgecolor', 'k');

%% Plotting

% Koordinatsystemer
trplot(T0C, 'frame', 'C', 'length', 0.2);       % Transportbånd
trplot(T0P, 'frame', 'P', 'length', 0.2);       % Pall
trplot(T0B, 'frame', 'B', 'length', 0.2);       % Eske
trplot(T0K, 'frame', 'K', 'length', 0.2);       % Kamera

% Innstillinger for plott
axis equal;

xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
title('6DOF Palletizing Robot Workspace')
view(3);
grid on;

% Tegner en robot

figure
Palletizing.plot(q0)
Palletizing.teach(q0)
