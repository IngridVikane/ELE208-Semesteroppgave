function generate_robot_description()
%generate_robot_description  Generate palletizer_robot/urdf/robot_description.urdf.xacro
%   from the project DH tables. Link lengths = the 12box setup with the
%   L2/L3 SWITCH (ELE208-Semesteroppgave/Robotarm & Workspace/
%   Palletizing_Robotarm.m, 10 Nov 2026):
%       L1=0.1, L2=0.9, L3=0.8, L4=0.2, L5=0.15, L6=0.15.
%   Link visuals/collisions are the UR5e meshes (meshes/ur5e), placed in our
%   link frames and rescaled so each mesh spans OUR segment joint_k ->
%   joint_{k+1} exactly (link 6 spans joint_6 -> tool0, incl. the 0.1 TCP
%   offset). Kinematics stay exactly per the DH tables below.
%
%   Standard DH per row j: A_j(q) = Rz(theta_j)*Tz(d_j)*Tx(a_j)*Rx(alpha_j).
%   The joint value commanded over ROS is EXACTLY the theta expression in their
%   table (offsets +pi/2 / -pi/2 INCLUDED; probe_dh.m proved numerically that
%   the shifted-origin chain below reproduces prod_j A_j(q) exactly, max err 0.0):
%       joint k <origin> (in parent link frame) = C_{k-1}   where C_j=Tz(d)Tx(a)Rx(alpha)
%       joint_1 origin = identity; TCP = frame6 + Tz(0.1) = link_6 + Tz(0.25)
%       (report eq.2, Powerpick 10 assumed offset; NO tool0 link — its fixed
%       joint gets lumped into link_6 by bullet-featherstone and inverts the
%       joint_6 velocity motor sign, see the link_6 block in make_xacro).
%   So the URDF chain(q) == prod_j A_j(table theta) identically. Re-checked on
%   500 random q vectors before writing; nothing is written unless it passes.
%
%   Mesh placement math (per link k, all asserted numerically at runtime):
%   both chains are pi/2-only, so A = Ro(k)'*Rt(k) (their link frame -> ours,
%   at q=0) and M0 = A*V (V = their visual-origin rpy rotation) are signed
%   permutations. T = S*M0 (S stretches our segment axis) decomposes exactly as
%   T = R*diag(scnat) with R a proper rotation; emitted URDF placement
%   p(n) = R*(scnat.*n) + t puts the mesh bbox center on the segment midpoint
%   and spans exactly [0, Lseg] (their visual origin offsets are absorbed by
%   the centering). Inertial = box from the placed bbox, mass from the user's
%   box-link model (Dynamics and Control/Masses.m: SOLID Al box links,
%   w=h=[0.05 0.05 0.05 0.02 0.02 0.02], rho=2700, V=L.*h.*w). Base link keeps
%   our own box (their base mesh unused).

    thisDir = fileparts(mfilename('fullpath'));
    pkgDir  = fullfile(thisDir, '..', 'vm', 'palletizer_robot');
    outFile = fullfile(pkgDir, 'urdf', 'robot_description.urdf.xacro');

    % ---- DH table; link lengths from the 12box setup (L2/L3 switched) ------
    L     = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15];   % Palletizing_Robotarm.m 10 Nov 2026
    d     = [L(1), 0,   0,   L(4), L(5), L(6)]; % d column (d5=L5, d6=L6)
    a     = [0,    L(2), L(3), 0,   0,   0];    % a column (a2=L2, a3=L3)
    alpha = [pi/2, 0,   0,   -pi/2, pi/2, 0];   % alpha column

    C = cell(1,6);
    for j = 1:6
        C{j} = eye(4);
        C{j}(1:3,1:3) = rx(alpha(j));
        C{j}(1:3,4) = [a(j); 0; d(j)]; %#ok<AGROW>
    end

    % ---- Verify chain identity on random poses before writing anything ----
    E4 = eye(4); E4(1:3,4) = [0;0;0.1];     % eq.2 TCP fold
    maxErr = 0;
    for k = 1:500
        q = 2*pi*rand(1,6) - pi;            % table theta VALUES as joint input
        Tdh = eye(4);                       % their DH product (= report closed form)
        for j = 1:6, Tdh = Tdh * rz4(q(j)) * C{j}; end %#ok<AGROW>
        pose = rz4(q(1));                   % URDF chain O_1=I, O_k=C_{k-1}
        for j = 2:6, pose = pose * C{j-1} * rz4(q(j)); end %#ok<AGROW>
        maxErr = max(maxErr, norm(pose*C{6}*E4 - Tdh*E4, 'fro'));
    end
    assert(maxErr < 1e-9, 'DH chain check failed: err=%g', maxErr);
    fprintf('DH-chain verification PASSED (max |chain-DH| = %.3e over 500 random poses)\n', maxErr);

    % ---- Origins to emit: O_1=I, O_k=C_{k-1} (j=2..6), tool0 = C6*Tz(0.1) --
    O = cell(1,7); O{1} = eye(4);
    for j = 2:6, O{j} = C{j-1}; end %#ok<AGROW>
    O{7} = C{6}; O{7}(1:3,4) = C{6}(1:3,4) + [0;0;0.1];

    o = mesh_placements(O);

    txt = make_xacro(O, o, L);
    fid = fopen(outFile,'w'); assert(fid~=-1,'cannot write %s',outFile);
    fwrite(fid, txt); fclose(fid);

    for j = 1:7
        p = O{j}(1:3,4).'; r = eulZYX(O{j}(1:3,1:3));
        assert(max(max(abs(rpyr(r) - O{j}(1:3,1:3)))) < 1e-9, 'O{%d}: rpy roundtrip failed', j);
        nm = 'base_joint'; if j>=2 && j<=6, nm=sprintf('joint_%d',j); end; if j==7, nm='tcp(=link6+Tz.25)'; end
        fprintf('%-12s xyz=(%.4f %.4f %.4f) rpy=(%.5f %.5f %.5f)\n', nm,p(1),p(2),p(3),r(1),r(2),r(3));
        if j <= 6
            fprintf('           mesh %-9s rpy=(%.6f %.6f %.6f) scale=[%.6f %.6f %.6f] mass=%.4f\n', ...
                o{j}.name, o{j}.rpy, o{j}.scale, o{j}.mass);
            fprintf('                        xyz=(%.6f %.6f %.6f) bboxdims=[%.4f %.4f %.4f]\n', ...
                o{j}.xyz, o{j}.dims);
        end
    end
    fprintf('Wrote %s\n', outFile);
end

function o = mesh_placements(O)
% UR5e meshes onto OUR link frames (see function header comment above).
names = {'shoulder','upperarm','forearm','wrist1','wrist2','wrist3'};
mass  = [0.675 6.075 5.4 0.216 0.162 0.162];    % Masses.m solid box-link masses (kg)
% their joint-origin rpy (ur5e default_kinematics.yaml) and the visual-origin
% rpy used for each mesh in their ur_macro.xacro:
tr = [0 0 0; pi/2 0 0; 0 0 0; 0 0 0; pi/2 0 0; pi/2 pi pi];
vr = [0 0 pi; pi/2 0 -pi/2; pi/2 0 -pi/2; pi/2 0 0; 0 0 0; pi/2 0 0];

mdir = fullfile(fileparts(mfilename('fullpath')), '..', 'vm', ...
    'palletizer_robot', 'meshes', 'ur5e');
o = cell(1,6);
Rt = rz(pi);                                    % their base_link->base_link_inertia (yaw pi)
for k=1:6
    Rt = Rt * rpyr(tr(k,:));                    % their link_k frame rotation at q=0
    Ro = eye(3); for j=2:k, Ro = Ro * O{j}(1:3,1:3); end   % our chain up to link k
    A  = Ro.' * Rt;                             % their link_k axes -> our link_k axes
    V  = rpyr(vr(k,:));
    M0 = A * V;                                 % native mesh coords -> our link frame
    for i=1:3                                   % assert signed permutation
        col = M0(:,i); nz = find(abs(col) > 0.5);
        assert(numel(nz)==1 && abs(abs(col(nz))-1) < 1e-9, ...
            'link %d: M0 not a signed permutation', k);
    end

    pts = stlread(fullfile(mdir,'collision',[names{k} '.stl'])).Points.';
    seg  = O{min(k+1,7)}(1:3,4);                % our segment vector in link_k frame
    if k==6, seg = seg - [0;0;0.1]; end         % wrist3 spans joint_6->flange (z 0..L6),
                                                % NOT ->TCP: keeps it out of the tool0 gripper box
    [Lseg, iax] = max(abs(seg));
    assert(nnz(abs(seg) > 1e-9)==1, 'link %d: segment not single-axis', k);
    n_i = find(abs(M0(iax,:)) > 0.5, 1);        % native axis landing on our segment axis
    s   = Lseg / (max(pts(n_i,:)) - min(pts(n_i,:)));

    S  = diag([1 1 1]); S(iax,iax) = s;
    T  = S * M0;                                % stretch along our segment axis
    % decompose T = R * diag(scnat) with R a proper rotation:
    P = zeros(3); v = zeros(1,3);
    for i=1:3
        r_ = find(abs(T(:,i)) > 0.5, 1);
        P(r_,i) = 1; v(i) = T(r_,i);
    end
    sig = sign(v);
    if det(P*diag(sig)) < 0, sig(n_i) = -sig(n_i); end
    R = P * diag(sig);
    scnat = v ./ sig;
    assert(max(max(abs(R*diag(scnat) - T))) < 1e-9, 'link %d: decomp failed', k);
    assert(abs(det(R)-1) < 1e-9, 'link %d: R not proper', k);

    % emitted placement: center the placed bbox on the segment midpoint
    mcT = (max(T*pts,[],2) + min(T*pts,[],2))/2;
    target = zeros(3,1); target(iax) = seg(iax)/2;
    t = target - mcT;
    p    = R * (scnat.' .* pts) + t;            % 3xN placed points
    ctr  = (max(p,[],2) + min(p,[],2))/2;
    span = max(p,[],2) - min(p,[],2);
    assert(max(abs(ctr - target)) < 1e-9, 'link %d: centering failed', k);
    assert(abs(span(iax) - Lseg) < 1e-9, 'link %d: span != Lseg', k);
    assert(max(max(abs(rpyr(eulZYX(R)) - R))) < 1e-9, 'link %d: rpy roundtrip failed', k);

    o{k} = struct('name', names{k}, 'xyz', t.', 'rpy', eulZYX(R), ...
        'scale', scnat, 'dims', span.', 'ctr', ctr.', 'mass', mass(k));
end
end

function txt = make_xacro(O, o, L)
    A = {};  % lines
    A{end+1} = '<?xml version="1.0" ?>'; %#ok<AGROW>
    A{end+1} = '<!-- AUTO-GENERATED by gazebo/control pc/generate_robot_description.m.'; %#ok<AGROW>
    A{end+1} = '     DH rows (Tabell 2): d=[L1,0,0,L4,L5,L6] a=[0,L2,L3,0,0,0]'; %#ok<AGROW>
    A{end+1} = sprintf('     alpha=[pi/2,0,0,-pi/2,pi/2,0], 12box link lengths L=[%.3g,%.3g,%.3g,%.3g,%.3g,%.3g] m.', L); %#ok<AGROW>
    A{end+1} = '     Joint k origin = C_{k-1}=Tz(d)Tx(a)Rx(alpha); joint_1 origin=identity;'; %#ok<AGROW>
    A{end+1} = '     tool0 fixed origin = C6*Tz(0.1): TCP = frame6 + 0.1 m along z6.'; %#ok<AGROW>
    A{end+1} = '     Commanded joint values are the table theta VALUES, +/-pi/2 offsets included.'; %#ok<AGROW>
    A{end+1} = '     Link meshes = UR5e meshes rescaled so each spans our joint_k -> joint_{k+1}.'; %#ok<AGROW>
    A{end+1} = '-->'; %#ok<AGROW>
    A{end+1} = '<robot name="palletizer_robot" xmlns:xacro="http://ros.org/wiki/xacro">'; %#ok<AGROW>
    A{end+1} = ''; %#ok<AGROW>
    A{end+1} = '  <xacro:include filename="$(find palletizer_robot)/urdf/inertial_macros.urdf.xacro"/>'; %#ok<AGROW>
    A{end+1} = '  <xacro:include filename="$(find palletizer_robot)/urdf/materials.urdf.xacro"/>'; %#ok<AGROW>
    A{end+1} = '  <xacro:include filename="$(find palletizer_robot)/urdf/robot_control.gazebo.xacro"/>'; %#ok<AGROW>
    A{end+1} = '  <xacro:include filename="$(find palletizer_robot)/urdf/camera.urdf.xacro"/>'; %#ok<AGROW>
    A{end+1} = ''; %#ok<AGROW>
    A{end+1} = '  <!-- 12box link lengths -->'; %#ok<AGROW>
    A{end+1} = '  <link name="base_footprint"/>'; %#ok<AGROW>
    A{end+1} = ''; %#ok<AGROW>
    A{end+1} = '  <!-- base_link frame == DH frame {0}. -->'; %#ok<AGROW>
    A{end+1} = '  <joint name="base_joint" type="fixed">'; %#ok<AGROW>
    A{end+1} = ['    <origin xyz="' f6(O{1}(1,4)) ' ' f6(O{1}(2,4)) ' ' f6(O{1}(3,4)) '" rpy="' f6(0) ' ' f6(0) ' ' f6(0) '"/>']; %#ok<AGROW>
    A{end+1} = '    <parent link="base_footprint"/><child link="base_link"/>'; %#ok<AGROW>
    A{end+1} = '  </joint>'; %#ok<AGROW>
    A{end+1} = ''; %#ok<AGROW>
    A{end+1} = '  <link name="base_link">'; %#ok<AGROW>
    A{end+1} = '    <visual><origin xyz="0 0 -0.08" rpy="0 0 0"/><geometry><box size="0.25 0.25 0.16"/></geometry><material name="silver"/></visual>'; %#ok<AGROW>
    A{end+1} = '    <collision><origin xyz="0 0 -0.08" rpy="0 0 0"/><geometry><box size="0.25 0.25 0.16"/></geometry></collision>'; %#ok<AGROW>
    A{end+1} = '    <xacro:inertial_box length="0.25" width="0.25" height="0.16" density="${7800/40}"><origin xyz="0 0 -0.08" rpy="0 0 0"/></xacro:inertial_box>'; %#ok<AGROW>
    A{end+1} = '  </link>'; %#ok<AGROW>

    for j = 1:6
        % joint_j: parent = previous link, origin O{j}
        if j == 1, par = 'base_link'; else, par = sprintf('link_%d_link', j-1); end
        A{end+1} = ''; %#ok<AGROW>
        A{end+1} = sprintf('  <joint name="joint_%d" type="revolute">', j); %#ok<AGROW>
        A{end+1} = ['    <origin xyz="' f6(O{j}(1,4)) ' ' f6(O{j}(2,4)) ' ' f6(O{j}(3,4)) '" rpy="' ...
            f6(eulZYX(O{j}(1:3,1:3))) '"/>']; %#ok<AGROW>
        A{end+1} = sprintf('    <parent link="%s"/><child link="link_%d_link"/>', par, j); %#ok<AGROW>
        A{end+1} = '    <axis xyz="0 0 1"/>'; %#ok<AGROW>
        A{end+1} = '    <limit effort="100" velocity="6.28" lower="-${pi}" upper="${pi}"/>'; %#ok<AGROW>
        A{end+1} = '    <dynamics damping="0.7" friction="1.0"/>'; %#ok<AGROW>
        A{end+1} = '  </joint>'; %#ok<AGROW>

        % link_j: UR5e mesh visual+collision, box inertial from placed bbox
        m = o{j};
        A{end+1} = ''; %#ok<AGROW>
        A{end+1} = sprintf('  <link name="link_%d_link">', j); %#ok<AGROW>
        A{end+1} = ['    <visual><origin xyz="' f6(m.xyz) '" rpy="' f6(m.rpy) '"/>' ...
                    '<geometry><mesh filename="package://palletizer_robot/meshes/ur5e/visual/' m.name '.dae"' ...
                    ' scale="' f6(m.scale) '"/></geometry></visual>']; %#ok<AGROW>
        A{end+1} = ['    <collision><origin xyz="' f6(m.xyz) '" rpy="' f6(m.rpy) '"/>' ...
                    '<geometry><mesh filename="package://palletizer_robot/meshes/ur5e/collision/' m.name '.stl"' ...
                    ' scale="' f6(m.scale) '"/></geometry></collision>']; %#ok<AGROW>
        if j == 6
            % PowerPick 10 gripper primitives baked into link_6 (TCP = link_6
            % + Tz(0.25), cup lip plane at link_6 z=0.25). NO tool0 fixed
            % joint: the bullet-featherstone fixed-joint lumping on this link
            % inverts the joint_6 velocity motor's effective sign and the Pro
            % position feedback runs away (observed: joint_6 ratcheted past
            % +/-1e4 rad). Joints 1-5 (no lumped children) are unaffected.
            A{end+1} = '    <visual><origin xyz="0 0 0.16" rpy="0 0 0"/><geometry><cylinder radius="0.027" length="0.02"/></geometry><material name="black"/></visual>'; %#ok<AGROW>
            A{end+1} = '    <visual><origin xyz="0 0 0.19" rpy="0 0 0"/><geometry><box size="0.045 0.045 0.04"/></geometry><material name="black"/></visual>'; %#ok<AGROW>
            A{end+1} = '    <visual><origin xyz="0 -0.075 0.22" rpy="0 0 0"/><geometry><box size="0.22 0.03 0.02"/></geometry><material name="black"/></visual>'; %#ok<AGROW>
            A{end+1} = '    <visual><origin xyz="0 0.075 0.22" rpy="0 0 0"/><geometry><box size="0.22 0.03 0.02"/></geometry><material name="black"/></visual>'; %#ok<AGROW>
            A{end+1} = '    <visual><origin xyz="0.06 -0.075 0.235" rpy="0 0 0"/><geometry><cylinder radius="0.018" length="0.03"/></geometry><material name="green"/></visual>'; %#ok<AGROW>
            A{end+1} = '    <visual><origin xyz="-0.06 -0.075 0.235" rpy="0 0 0"/><geometry><cylinder radius="0.018" length="0.03"/></geometry><material name="green"/></visual>'; %#ok<AGROW>
            A{end+1} = '    <visual><origin xyz="0.06 0.075 0.235" rpy="0 0 0"/><geometry><cylinder radius="0.018" length="0.03"/></geometry><material name="green"/></visual>'; %#ok<AGROW>
            A{end+1} = '    <visual><origin xyz="-0.06 0.075 0.235" rpy="0 0 0"/><geometry><cylinder radius="0.018" length="0.03"/></geometry><material name="green"/></visual>'; %#ok<AGROW>
        end
        A{end+1} = '    <inertial>'; %#ok<AGROW>
        if j == 6
            % combined inertial: mesh box + 1.0 kg gripper box (both aligned
            % with link_6 axes; z-offsets only -> parallel-axis on ixx/iyy)
            mg = 1.0; cg = [0 0 0.20]; dg = [0.24 0.195 0.10];
            M6 = m.mass + mg;
            c6 = (m.mass*m.ctr + mg*cg) / M6;
            i1 = (m.mass/12).*[m.dims(2)^2+m.dims(3)^2, m.dims(1)^2+m.dims(3)^2, m.dims(1)^2+m.dims(2)^2];
            i2 = (mg/12).*[dg(2)^2+dg(3)^2, dg(1)^2+dg(3)^2, dg(1)^2+dg(2)^2];
            dz1 = m.ctr(3) - c6(3); dz2 = cg(3) - c6(3);
            I6 = i1 + m.mass*dz1^2.*[1 1 0] + i2 + mg*dz2^2.*[1 1 0];
            A{end+1} = ['      <origin xyz="' f6(c6) '" rpy="0 0 0"/>']; %#ok<AGROW>
            A{end+1} = sprintf('      <mass value="%.6f"/>', M6); %#ok<AGROW>
            A{end+1} = ['      <inertia ixx="' f9(I6(1)) '" ixy="0.0" ixz="0.0" iyy="' f9(I6(2)) ...
                        '" iyz="0.0" izz="' f9(I6(3)) '"/>']; %#ok<AGROW>
        else
            A{end+1} = ['      <origin xyz="' f6(m.ctr) '" rpy="0 0 0"/>']; %#ok<AGROW>
            A{end+1} = sprintf('      <mass value="%.6f"/>', m.mass); %#ok<AGROW>
            ixx = m.mass/12*(m.dims(2)^2 + m.dims(3)^2);
            iyy = m.mass/12*(m.dims(1)^2 + m.dims(3)^2);
            izz = m.mass/12*(m.dims(1)^2 + m.dims(2)^2);
            A{end+1} = ['      <inertia ixx="' f9(ixx) '" ixy="0.0" ixz="0.0" iyy="' f9(iyy) ...
                        '" iyz="0.0" izz="' f9(izz) '"/>']; %#ok<AGROW>
        end
        A{end+1} = '    </inertial>'; %#ok<AGROW>
        A{end+1} = '  </link>'; %#ok<AGROW>
    end

    A{end+1} = ''; %#ok<AGROW>
    A{end+1} = '  <!-- TCP frame (eq.2) = link_6 origin + 0.1 m along z6 = link_6 + Tz(0.25).'; %#ok<AGROW>
    A{end+1} = '       No tool0 link: its fixed joint would be lumped into link_6 by the'; %#ok<AGROW>
    A{end+1} = '       bullet-featherstone plugin and invert the joint_6 motor sign. -->'; %#ok<AGROW>

    % ---- camera (course pattern, NOT generated - see camera.urdf.xacro) -----
    A{end+1} = '';
    A{end+1} = '  <xacro:camera_sensor parent_link="base_footprint" x="0" y="-1.0" z="1.8"';
    A{end+1} = '      roll="0" pitch="1.5708" yaw="0"/>';
    A{end+1} = '';
    A{end+1} = '</robot>';

    txt = [strjoin(A, newline) newline];
end

function r = rx(a)
    c = cos(a); s = sin(a);
    r = [1 0 0; 0 c -s; 0 s c];
end

function r = rz(a)
    c = cos(a); s = sin(a);
    r = [c -s 0; s c 0; 0 0 1];
end

function T = rz4(a)
% homogeneous Rz
    T = eye(4); T(1:3,1:3) = rz(a);
end

function R = rpyr(rpy)
% URDF fixed-axis rpy: R = Rz(yaw)*Ry(pitch)*Rx(roll)
    cr = cos(rpy(1)); sr = sin(rpy(1));
    cp = cos(rpy(2)); sp = sin(rpy(2));
    cy = cos(rpy(3)); sy = sin(rpy(3));
    Rx = [1 0 0; 0 cr -sr; 0 sr cr];
    Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
    Rz = [cy -sy 0; sy cy 0; 0 0 1];
    R = Rz*Ry*Rx;
end

function e = eulZYX(R)
% URDF rpy extraction: R = Rz(yaw)*Ry(pitch)*Rx(roll) -> e = [roll pitch yaw]
    e = [atan2(R(3,2), R(3,3)), atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2)), atan2(R(2,1), R(1,1))];
end

function s = f6(v)
% scalar -> "x" with 6 decimals, -0 normalized, row vector -> "a b c"
    v = round(v*1e6)/1e6;
    v(v==0) = 0;
    s = strtrim(sprintf('%.6f ', v));
end

function s = f9(v)
    v = round(v*1e9)/1e9;
    v(v==0) = 0;
    s = strtrim(sprintf('%.9f ', v));
end
