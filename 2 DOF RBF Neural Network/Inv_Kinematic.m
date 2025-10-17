clear all
clc
%% Parameters
x0 = sqrt(2)/2;
y0 = sqrt(2)/2;
l1 = 1;
l2 = 0.5;
T=5;
R = 0.5;
w = 2*pi/T;

%% Inversed Kinematic
syms clk;
xd = x0 + R*cos(w*clk +pi/2);
yd = y0 + R*sin(w*clk +pi/2);

qd2 = acos((xd^2 + yd^2 - l1^2 - l2^2)/(2*l1*l2));
qd2_dot = diff(qd2);
qd2_dotdot = diff(qd2_dot);

qd1 = asin((yd*(l1+l2*cos(qd2))-xd*l2*sin(qd2))/(xd^2+yd^2));
qd1_dot = diff(qd1);
qd1_dotdot = diff(qd1_dot);