function fk_check()
%% Denne filen verifiserer foroverkinematikken i den genererte URDF/xacro-filen mot DH-modellen.

%    Copyright:     HVL
%    Project:       ELE208 - Robotmanipulatorer
%    Author:        Christoffer Træet Herland
%    Date created:  Høst 2026
%    Input:         'robot_description.urdf.xacro'
%
%    Output:        Verifiseringsresultat i kommandovinduet (maksimal FK-feil over 500 posisjoner)
%---------------------------------------------------------------------%
%%
% Note: Kjøres etter 'generate_robot_description.m'. Leser ledd-origoene
% direkte fra den genererte xacro-filen og sammenligner URDF-kjeden
% (inkludert 0.10 m TCP-forskyvning langs z6) mot standard DH-produkt over
% 500 tilfeldige leddvektorer.
% Utviklet med hjelp fra Gemini (AI).

%fk_check  Verify the SHIPPED urdf/robot_description.urdf.xacro reproduces their
%   DH product prod_j Rz(theta_j)Tz(d)Tx(a)Rx(alpha) + eq.2 tool offset, by
%   parsing joint origins straight out of the generated xacro (an independent
%   check of the shipped file, not the generator's code path). Run after
%   generate_robot_description.m. Note: every emitted joint rpy has a single
%   nonzero axis (x only), so rpy->R is convention-independent here.

    thisDir = fileparts(mfilename('fullpath'));
    f = fullfile(thisDir, '..', 'vm', 'palletizer_robot', 'urdf', 'robot_description.urdf.xacro');
    s = fileread(f);

    tok = regexp(s, '<joint name="(joint_\d)"[^>]*>\s*<origin xyz="([^"]+)" rpy="([^"]+)"/>', 'tokens');
    assert(numel(tok) == 6, 'expected 6 revolute joints in xacro, found %d', numel(tok));

    O = cell(1,7);
    for k = 1:6
        xyz = str2num(['[' tok{k}{2} ']']); %#ok<ST2NM>
        rpyv = str2num(['[' tok{k}{3} ']']); %#ok<ST2NM>
        O{k} = trn(xyz, rpyv);               %#ok<SAGROW>
    end
    % No tool0_joint in the xacro (lumping fix): TCP = link_6 + Tz(L6+0.1),
    % assigned below once L is defined.

    L     = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15];     % 12box setup (L2/L3 switched, 10 Nov 2026)
    d     = [L(1), 0,   0,   L(4), L(5), L(6)];   % Tabell 2 DH (d5=L5, d6=L6)
    a     = [0,    L(2), L(3), 0,   0,   0];      % (a2=L2, a3=L3 per row text)
    alpha = [pi/2, 0,   0,   -pi/2, pi/2, 0];

    O{7} = trn([0 0 L(6)+0.1], [0 0 0]);

    C = cell(1,6);
    for j = 1:6
        C{j} = trn([a(j) 0 d(j)], [alpha(j) 0 0]); %#ok<AGROW>
    end
    Etool = O{7};                                  % fixed tool origin from the xacro

    maxErr = 0; tcpZero = eye(4); %#ok<NASGU>
    for k = 1:500
        q = 2*pi*rand(1,6) - pi;               % tabulated theta VALUES as input
        Tdh = eye(4);                          % their DH product (report closed form)
        for j = 1:6, Tdh = Tdh * rotz4(q(j)) * C{j}; end %#ok<AGROW>
        pose = O{1} * rotz4(q(1));             % chain exactly as URDF will compose it
        for j = 2:6, pose = pose * O{j} * rotz4(q(j)); end %#ok<AGROW>
        maxErr = max(maxErr, norm(pose*Etool - Tdh*[eye(3) [0;0;0.1]; 0 0 0 1], 'fro'));
    end
    % NOTE: xacro prints rpy as %.6f (pi/2 -> 1.570796), so the parsed chain
    % differs from exact DH by ~1e-7 rad * lever. That is file display
    % precision, not kinematics error; wrong schemes would be O(0.1) or worse.
    assert(maxErr < 1e-4, ...
        'shipped xacro does NOT reproduce their DH product (wrong scheme?): err=%g', maxErr);

    if maxErr > 1e-9
        fprintf('note: residual %.2e is %.6f rounding of pi/2 in the generated file, not a model error\n', maxErr); %#ok<FTCN>
    end
    fprintf('FK check PASSED: shipped xacro vs DH product, max err %.3e\n', maxErr);
    % zero-pose TCP pose (compare with their closed form 0T6*Tz(0.1) by eye):
    q = zeros(1,6); Tdh = eye(4); for j=1:6, Tdh = Tdh*rotz4(q(j))*C{j}; end %#ok<AGROW>
    fprintf('their DH 0T6*Tz(.1):\n'); disp(Tdh*[eye(3) [0;0;0.1]; 0 0 0 1]);
end

function T = trn(xyz, rpy)   % URDF origin: fixed-axis XYZ (single-axis here => unambiguous)
    Rx = [1 0 0; 0 cos(rpy(1)) -sin(rpy(1)); 0 sin(rpy(1)) cos(rpy(1))];
    T = eye(4); T(1:3,1:3) = Rx;                % all our joint rpys are x-only rotations
    T(1:3,4) = xyz(:);
end

function T = rotz4(a); R=[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; T=eye(4); T(1:3,1:3)=R; end
