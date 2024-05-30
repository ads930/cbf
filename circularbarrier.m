clc, clear, close all

%% Destination

% simple wall
% xref = [60, 60];
% L = [120, 40];   
% xx0 = [0.1;0.1;0.1;pi/2];

% circular wall
xref = [90, 0];
L = [50, 120, 0, 0]; 
xx0 = [-90;0.01;0.01;0.01]; 


%% Simulation
dt = 0.01;
tt = 0:dt:20;

%% Simulation
[t, x] = ode45(@(t, x) differential_drive(t, x, xref, L), tt, xx0);
disp([x(end,1), x(end, 2)])

figure(1), subplot(211), plot(t, x(:, 1), t, x(:, 2), t, x(:, 4), LineWidth=2)
legend(["x", "y", "\theta"]); title("States");
subplot(212), plot(t(1:end-1), diff(x(:, 1)), t(1:end-1), diff(x(:, 2)), LineWidth=2)
legend(["Acceleration", "Angular Rate"]); title("Inputs");

figure(2), plot(x(:, 1), x(:, 2), 'g', LineWidth=2); hold on;
plot(xref(1), xref(2), 'or', xx0(1), xx0(2), 'ob', MarkerSize=5, LineWidth=3)
hold on
% Planar wall
% plot([L(1), -L(1), -L(1), L(1), L(1)], [L(2), L(2), -L(2), -L(2), L(2)], 'r--', LineWidth=1), axis equal;
% title("Rectangular wall")
% legend(["Trajectory","Destination", "Starting Point", "Barrier"])
% Circular wall
theta = linspace(0, 2 * pi, 100);
plot(L(3) + L(2) * cos(theta), L(4) + L(2) * sin(theta), 'r--', L(3) + L(1) * cos(theta), L(4) + L(1) * sin(theta), 'b--', LineWidth=1), axis equal;
title("Circular road")
legend(["Trajectory","Destination", "Starting Point", "Outer Barrier", "Inner Barrier"])

%% Plant and Controller
function dxdt= differential_drive(t, x, xref, L)

k = 1;
gamma1 = 500;
gamma2 = 500;

k1 = 1;
k2 = 2;
kv = 2;

L1 = L(1); L2 = L(2);
C1 = L(3); C2 = L(4);
x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4);

% Simple barrier at destination
% del_psi_f = gamma1*((x2^2-L2^2)*x1*x3*cos(x4) + (x1^2-L1^2)*x2*x3*sin(x4)) + 4*x1*x2*x3^2*sin(x4)*cos(x4) + x3^2*((x2^2-L2^2)*cos(x4)^2 + (x1^2-L1^2)*sin(x4)^2);
% alpha_2 = gamma2*((x2^2-L2^2)*x1*x3*cos(x4) + (x1^2-L1^2)*x2*x3*sin(x4) + gamma1/2*(x1^2*x2^2 + L1^2*L2^2 - x1^2*L2^2 - x2^2*L1^2));
% numerator = del_psi_f + alpha_2;
% denom1 = (x2^2-L2^2)*x1*cos(x4) + (x1^2-L1^2)*x2*sin(x4);
% denom2 = -x1*x3*sin(x4)*(x2^2-L2^2) + x2*x3*cos(x4)*(x1^2-L1^2);

% Circular barrier with offset using symbolic toolbox
numerator = gamma2*(x3*cos(x4)*((2*C1 - 2*x1)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2) + (C1 - x1)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2)) + x3*sin(x4)*((2*C2 - 2*x2)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2) + (C2 - x2)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2)) - gamma1*((C1 - x1)^2 - L1^2 + (C2 - x2)^2)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2)) - x3*cos(x4)*(x3*cos(x4)*(2*(C1 - x1)*(2*C1 - 2*x1) - L2^2 - L1^2 + 2*(C1 - x1)^2 + 2*(C2 - x2)^2) - gamma1*(C1 - x1)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2) - gamma1*(2*C1 - 2*x1)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2) + x3*sin(x4)*((C1 - x1)*(2*C2 - 2*x2) + (C2 - x2)*(2*C1 - 2*x1))) - x3*sin(x4)*(x3*sin(x4)*(2*(C2 - x2)*(2*C2 - 2*x2) - L2^2 - L1^2 + 2*(C1 - x1)^2 + 2*(C2 - x2)^2) - gamma1*(C2 - x2)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2) + x3*cos(x4)*((C1 - x1)*(2*C2 - 2*x2) + (C2 - x2)*(2*C1 - 2*x1)) - gamma1*(2*C2 - 2*x2)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2));
denom1 = cos(x4)*((2*C1 - 2*x1)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2) + (C1 - x1)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2)) + sin(x4)*((2*C2 - 2*x2)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2) + (C2 - x2)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2));
denom2 = x3*cos(x4)*((2*C2 - 2*x2)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2) + (C2 - x2)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2)) - x3*sin(x4)*((2*C1 - 2*x1)*((C1 - x1)^2/2 - L2^2/2 + (C2 - x2)^2/2) + (C1 - x1)*((C1 - x1)^2 - L1^2 + (C2 - x2)^2));

ang_error = atan2((xref(2) - x(2)),(xref(1) - x(1))) - (mod(x(4) + pi, 2*pi) - pi);
% pos_error = norm(xref-[x(1), x(2)]);

dx1 = x(3)*cos(x(4));
dx2 = x(3)*sin(x(4));

if numerator >=0
    % dx3 = k1*pos_error*(pos_error>0.2);
    % dx3 = -kv*x(3) - (x(1)-xref(1))*cos(x(4)) - k1*(x(2)-xref(2))*sin(x(4));
    dx3 = -kv*x(3) - (x(1)-xref(1))*cos(-ang_error) - k1*(x(2)-xref(2))*sin(-ang_error);
    dx4 = k2*ang_error;
else
    dx3 = -numerator/norm([denom1 denom2])^2 * denom1;
    dx4 = -numerator/norm([denom1 denom2])^2 * denom2;
end

dxdt = [dx1; dx2; dx3; dx4];
end

