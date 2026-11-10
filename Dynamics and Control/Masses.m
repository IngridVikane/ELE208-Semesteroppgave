% Link lengths [m]
L = [0.10 0.90 0.80 0.20 0.15 0.15];

% Width [m]
b = [0.10 0.10 0.10 0.04 0.04 0.04];

% Wall thickness [m]
t = 0.005;

% Aluminium density [kg/m^3]
p = 2700;

% Volume [m^3]
V = L .* (b.^2 - (b - 2*t).^2);

% Link masses [kg]
m = p .* V;

for i = 1:6
    fprintf('Link %d: %.3f kg\n', i, m(i));
end

% Total mass
m_total = sum(m);

disp(m)
disp(m_total)