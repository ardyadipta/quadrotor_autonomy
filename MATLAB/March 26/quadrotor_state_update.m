function x_dot = quadrotor_state_update(t, x, u)
% Parameters
kF = 6.11e-8;   % N/(rpm)^2
kM = 1.5e-9;    % Nm/(rpm)^2
km = 20;        % s-1
g = 9.8;
Ix = 0.005;
Iy = 0.005;
Iz = 0.005;
I = [Ix 0 0; 0 Iy 0; 0 0 Iz];
L = 0.17933; % Length (distance from prop axis to z_b): 179.33mm
m = 0.6; % Mass

% State
pos = x(1 : 3);
d_pos = x(4 : 6);
euler_zxy = x(7 : 9);
wb = x(10 : 12);
omega = x(13 : 16);

% Update
mixer = [kF kF kF kF; 0 L * kF 0 -L * kF;...
    -L * kF 0 L * kF 0; kM -kM kM -kM];
uc = mixer * (omega .^ 2);
F = uc(1);
M = uc(2 : 4);

phi = euler_zxy(1);
theta = euler_zxy(2);
psi = euler_zxy(3);
R_Z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R_X = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R_Y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R = R_Z * R_X * R_Y;

dd_pos = 1 / m * R * [0; 0; F] + [0; 0; -g];
d_wb = inv(I) * (M - cross(wb, I * wb));

d_euler_zxy = inv([cos(theta) 0 -cos(phi) * sin(theta); 0 1 sin(phi); sin(theta) 0 cos(phi) * cos(theta)]) * wb;

omega_des = u(1 : 4);
d_omega = -km * (omega - omega_des);

x_dot = [d_pos; dd_pos; d_euler_zxy; d_wb; d_omega];