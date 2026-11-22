% Lenkelengder [m]
L = [0.10 0.90 0.80 0.20 0.15 0.15];

% Bredde [m]
w = [0.05 0.05 0.05 0.02 0.02 0.02];

h = [0.05 0.05 0.05 0.02 0.02 0.02];

% Aluminiums tetthet [kg/m^3]
p = 2700;

% Volum [m^3]
V = L.*h.*w

% Lenke masser [kg]
m = p .* V;

for i = 1:6
    fprintf('Link %d: %.3f kg\n', i, m(i));
end

% Total masse
m_total = sum(m);

disp(m)
disp(m_total)