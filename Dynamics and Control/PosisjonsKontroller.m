%% Dynamics and Control
% 
%    Copyright:     HVL
%    Project:       ELE208 - Robotmanipulatorer
%    Author:        Erlend Østbø
%       
%    Date created:  Høst 2026
%   Input: Palletizing_Robotarm.m, pallet_cycles.mat
%
%   Output: Dynamisk modell og PID-regulering av ledd 2

% Simulerer et posisjonssprang på 30 grader og banefølging
% for ledd 2 ved hjelp av en PID-posisjonsregulator.
% Simuleringen inkluderer griperen, men uten eske.

% - Griperens masse på 2 kg er inkludert.
% - Eskens masse på 10 kg er ikke inkludert.
%---------------------------------------------------------------------% 
clc;
clear;
close all;

%% 1. Import av robotmodell og banedata

scriptPath = fileparts(mfilename('fullpath'));
projectPath = fileparts(scriptPath);
addpath(genpath(projectPath));

run('Palletizing_Robotarm.m');

%load(fullfile(projectPath,'Trajectory Planning 12box','pallet_cycles.mat'));
load('pallet_cycles.mat');

%% 2. Dynamiske parametere

% Lenkelengder og bredde [m]
L = [0.10 0.90 0.80 0.20 0.15 0.15];
w = [0.10 0.10 0.10 0.04 0.04 0.04];
h = w;

% Veggtykkelse [m] og aluminiumstetthet [kg/m^3]
thickness = 0.005;
rho = 2700;

% Innvendige dimensjoner
wi = w - 2*thickness;
hi = h - 2*thickness;

% Volum og masse for hule aluminiumsprofiler
V = L.*(w.*h - wi.*hi);
m = rho.*V;

Palletizing.gravity = [0 0 9.81];

% Massesenter i lokale koordinatsystemer
r = [ 0       -L(1)/2  0;
    -L(2)/2   0       0;
    -L(3)/2   0       0;
    0        L(4)/2  0;
    0       -L(5)/2  0;
    0        0      -L(6)/2];

% Ytre dimensjoner langs lokale x-, y- og z-akser
dim = [w(1) L(1) h(1);
    L(2) w(2) h(2);
    L(3) w(3) h(3);
    w(4) L(4) h(4);
    w(5) L(5) h(5);
    w(6) h(6) L(6)];

% Innvendige dimensjoner
dim_inner = [wi(1) L(1)  hi(1);
    L(2)  wi(2) hi(2);
    L(3)  wi(3) hi(3);
    wi(4) L(4)  hi(4);
    wi(5) L(5)  hi(5);
    wi(6) hi(6) L(6)];

% Aktuatorparametere
Jm = 1e-4;      % Antatt rotortreghet [kg*m^2]
Bm = 0;         % Viskøs friksjon neglisjeres
Tc = [0 0];     % Coulomb-friksjon neglisjeres
G  = 80;        % Girutveksling

for i = 1:6

    % Ytre dimensjoner
    x = dim(i,1);
    y = dim(i,2);
    z = dim(i,3);

    % Innvendige dimensjoner
    xi = dim_inner(i,1);
    yi = dim_inner(i,2);
    zi = dim_inner(i,3);

    % Masse av ytre prisme og hulrom
    mo = rho*x*y*z;
    mi = rho*xi*yi*zi;

    % Treghetstensor for ytre prisme
    Io = mo/12 * diag([y^2+z^2, ...
        x^2+z^2, ...
        x^2+y^2]);

    % Treghetstensor for hulrom
    Ii = mi/12 * diag([yi^2+zi^2, ...
        xi^2+zi^2, ...
        xi^2+yi^2]);

    % Tildeler dynamiske parametere til robotmodellen
    Palletizing.links(i).m  = m(i);
    Palletizing.links(i).r  = r(i,:);
    Palletizing.links(i).I  = Io-Ii;
    Palletizing.links(i).Jm = Jm;
    Palletizing.links(i).B  = Bm;
    Palletizing.links(i).Tc = Tc;
    Palletizing.links(i).G  = G;
end

%% 3. Linearisert motormodell for ledd 2

% Modellforutsetninger:
% - Konstant treghetsmoment rundt startkonfigurasjonen.
% - Dynamisk kobling mellom ledd neglisjeres.
% - Tyngdekraften antas kompensert separat.
% - Elektrisk motordynamikk og friksjon neglisjeres.
% - Ideell girutveksling uten tap eller slark.
% - Eskens masse er ikke inkludert.
% - Moment- og hastighetsbegrensninger neglisjeres.

joint = 2;                  % Ledd 2 velges
q0 = cycles(1).q(1,:);


% Griperens masse [kg] og massesenter [m]
Palletizing.payload(2, [0 0 0.05]);

% Treghetsmatrise inkludert reflektert rotortreghet
M = Palletizing.inertia(q0);

% Ekvivalente parametere referert til motorsiden
Jeq = M(joint,joint)/G^2;
Beq = Bm;

% Transferfunksjon fra motormoment til leddposisjon
s = tf('s');
P = 1/(G*s*(Jeq*s + Beq))

%% 4. PID-regulator med derivatfilter

wc = 30;    % Ønsket kryssfrekvens [rad/s]
pm = pidtuneOptions('PhaseMargin',70);

% Automatisk dimensjonering
C = pidtune(P,'PIDF',wc,pm);

% Manuell finjustering for å redusere oversving
C.Ki = C.Ki*0.3;
C.Kd = C.Kd*1.2;

% Lukket reguleringssløyfe
T = feedback(C*P,1);

% Stabilitetsanalyse
[~,Pm,~,Wcp] = margin(C*P);

fprintf('Stabil regulator: %d\n',isstable(T));
fprintf('Fasemargin: %.2f grader\n',Pm);

%% 5. Sprangrespons

q_step = deg2rad(30);

figure;
step(rad2deg(q_step)*T);
grid on;
title('Sprangrespons for ledd 2');
xlabel('Tid [s]');
ylabel('Vinkelendring [grader]');

info = stepinfo(q_step*T);

fprintf('Innsvingningstid: %.3f s\n',info.SettlingTime);
fprintf('Oversving: %.2f %%\n',info.Overshoot);
fprintf('Stasjonært avvik: %.4f grader\n', ...
    rad2deg(abs(q_step*(1-dcgain(T)))));

%% 6. Banefølging

% Fjerner dupliserte tidspunkter
[t,idx] = unique(cycles(1).t(:));
t = t-t(1);

% Jevn tidsdiskretisering for lsim()
t_sim = linspace(0,t(end),length(t))';

% Interpolerer ønsket leddposisjon
q_ref = interp1(t,cycles(1).q(idx,joint),t_sim);

% Simulerer posisjonsregulatoren
q_relative = q_ref-q_ref(1);
q_sim = lsim(T,q_relative,t_sim)+q_ref(1);

figure;
plot(t_sim,rad2deg(q_ref),'--','LineWidth',1.5);
hold on;
plot(t_sim,rad2deg(q_sim),'LineWidth',1.5);
grid on;

title('Posisjonsregulering av ledd 2');
xlabel('Tid [s]');
ylabel('Leddvinkel [grader]');
legend('Ønsket posisjon','Simulert posisjon');

%% 7. Posisjonsavvik

e = rad2deg(q_ref-q_sim);

figure;
plot(t_sim,e,'LineWidth',1.5);
hold on;
yline(1,'r--');
yline(-1,'r--');
grid on;

title('Posisjonsavvik for ledd 2');
xlabel('Tid [s]');
ylabel('Posisjonsavvik [grader]');

fprintf('Maksimalt posisjonsavvik: %.4f grader\n',max(abs(e)));

%% 8. Regulatorens momentbehov

% Regulatorens utgang representerer motormoment
tau_motor = lsim(feedback(C,P),q_relative,t_sim);

% Konverterer motormoment til leddmoment
tau_joint = G*tau_motor;