clc
clear
close all
%script that finds the barrier function that we need
syms x1 x2 x3 x4 L1 L2 C1 C2 theta real
syms gamma1 gamma2 real
syms k real

% h = 0.5*(L1^2 - x1^2)*(L2^2 - x2^2);
h = 0.5*(L2^2 - ((x1-C1)^2 + (x2-C2)^2))*(((x1-C1)^2 + (x2-C2)^2) - L1^2);
f = [x3*cos(x4);x3*sin(x4);0;0];

g1 = [0;0;1;0];
g2 = [0;0;0;1];

psi1 = [diff(h, x1) diff(h, x2) diff(h, x3) diff(h, x4)] * f + gamma1*h;
psi2 = [diff(psi1, x1) diff(psi1, x2) diff(psi1, x3) diff(psi1, x4)] * f + gamma2*psi1;
psi2 = simplify(psi2) 

denom1 = [diff(psi1, x1) diff(psi1, x2) diff(psi1, x3) diff(psi1, x4)] * g1
denom2 = [diff(psi1, x1) diff(psi1, x2) diff(psi1, x3) diff(psi1, x4)] * g2
