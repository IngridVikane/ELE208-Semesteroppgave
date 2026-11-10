clc; 
clear; 

% Run model & workspace
run('Defining_Palletizing_Robot.m');

% Definerer leddvinkler
%                   q1      q2      q3      q4      q5      q6
q_liste = [ ...
                    -60     -10     30      100     0       30;
                    -75     -20     -40.1   92.8    -180    0; ...
                    -7      20      -40.1   92.8    180     0 ...
        ];

%Konverterer fra grader til radianner
q_liste_rad = deg2rad(q_liste);


for i = 1:size(q_liste_rad,1)

    % Hent leddstillinger i listen
    q = q_liste_rad(i,:);

    
    % Beregner Jacobi-matrise
    J = Palletizing.jacob0(q);
    title(sprintf('Posisjon q%d', i));

    disp('Jacobi-matrise:')
    disp(J)

    % Sjekker om robotens bevegelse er begrenset
    r = rank(J);
    detJ = det(J);

    
    fprintf('Posisjon %d:\n', i);
    % Kontroller singularitet
    if r < 6
        disp('Singulær posisjon');
    else
        disp('Ikke singulæ');
    end

    % Printer resultatet
    
    fprintf('q = %s grader\n', mat2str(round(rad2deg(q),1)));
    fprintf('Rank = %d\n', r);
    fprintf('Determinant = %d\n', detJ);
    fprintf('\n');
end

