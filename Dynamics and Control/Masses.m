% Link lengths [m]
L = [0.10 0.90 0.80 0.20 0.15 0.15];

% Width [m]
w = [0.05 0.05 0.05 0.02 0.02 0.02];

h = [0.05 0.05 0.05 0.02 0.02 0.02];

% Aluminium density [kg/m^3]
p = 2700;

% Volume [m^3]
V = L.*h.*w

% Link masses [kg]
m = p .* V;

for i = 1:6
    fprintf('Link %d: %.3f kg\n', i, m(i));
end

% Total mass
m_total = sum(m);

disp(m)
disp(m_total)