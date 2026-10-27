clc; 
clear; 

% Run model & workspace
run('Defining_Palletizing_Robot.m');

% Definerer leddvinkler
q = deg2rad([180 -50 0 0 90 90]);

% Endrer robot til posisjon
Palletizing.teach(q)

% Beregner Jacobi matrise
J = Palletizing.jacob0(q);

disp('Jacobi-matrise:')
disp(J)

% Sjekker Rank. < 6 Robotens bevegelse er begrenset
disp('Rank:')
disp(rank(J))

% Sjekker om determinant er 0
disp('Determinant:')
disp(det(J))

% Plotter robot med akser
figure;
Palletizing.plot(q);
title('Robotkonfigurasjon');
hold on;

[~,T] = Palletizing.fkine(q);

for i = 1:Palletizing.n
    trplot(T(i).T, 'frame',num2str(i), 'rgb', 'lenght', 0,15);
end
hold off;