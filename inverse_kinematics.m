function [Theta1, Theta2, Theta3] = inverse_kinematics(x, y, phi)
% This function calculates the inverse kinematics for a 3-DOF planar robot
%
% INPUTS:
%   x, y: The target coordinates of the end-effector.
%   phi:  The target orientation of the end-effector in radians.
%
% OUTPUTS:
%   Theta1, Theta2, Theta3: The required joint angles in radians.

% Define link lengths
L1 = 1; % meters
L2 = 1; % meters
L3 = 1; % meters

% Determine the Wrist Center Coordinates (Xwr, Ywr)
Xwr = x - L3 * cos(phi);
Ywr = y - L3 * sin(phi);

% Calculate the Second Joint Angle (Theta2)
cos_theta2 = (Xwr^2 + Ywr^2 - L1^2 - L2^2) / (2 * L1 * L2);

% Check if the calculated value for cos(theta2) is valid.
% If not, the target point is unreachable.
if abs(cos_theta2) > 1
    error('Target position is outside the robot''s reachable workspace.');
end

% Calculate sin(theta2) for the "elbow up" configuration (positive value)
sin_theta2 = sqrt(1 - cos_theta2^2);
Theta2 = atan2(sin_theta2, cos_theta2);

% Calculate the First Joint Angle (Theta1)
k1 = L1 + L2 * cos(Theta2);
k2 = L2 * sin(Theta2);
Theta1 = atan2(Ywr, Xwr) - atan2(k2, k1);

% Calculate the Third Joint Angle (Theta3) 
Theta3 = phi - Theta1 - Theta2;

end
