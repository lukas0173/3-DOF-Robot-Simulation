function [X, Y] = forward_kinematics(theta1, theta2, theta3)
% This function calculates the forward kinematics for a 3-DOF planar robot
%
% INPUTS:
%   theta1, theta2, theta3: Joint angles in radians.
%
% OUTPUTS:
%   X, Y: The coordinates of the end-effector.

% Define link lengths
L1 = 1; % meters
L2 = 1; % meters
L3 = 1; % meters

% Forward Kinematics Equations 

X = L1*cos(theta1) + L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3);
Y = L1*sin(theta1) + L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3);

end
