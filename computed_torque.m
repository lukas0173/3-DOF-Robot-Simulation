function [Torque1, Torque2, Torque3] = computed_torque(q1, q2, q3, q1_dot, q2_dot, q3_dot, qd1, qd2, qd3, qd1_dot, qd2_dot, qd3_dot, qd1_dotdot, qd2_dotdot, qd3_dotdot)
% Computes the required joint torques for a 3-DOF planar manipulator using a computed torque controller.
% The dynamic model is based on the Lagrange formulation.

%% Robot Parameters
m1 = 2;     m2 = 1.5;   m3 = 1; % Mass of each link (kg)
l1 = 0.2;   l2 = 0.5;   l3 = 0.3; % Length of each link (m)
r1 = l1/2;  r2 = l2/2;  r3 = l3/2; % Center of mass distance for each link (m)
I1 = 1/12*m1*l1^2; % Moment of inertia for link 1
I2 = 1/12*m2*l2^2; % Moment of inertia for link 2
I3 = 1/12*m3*l3^2; % Moment of inertia for link 3
g = 9.81; % Acceleration due to gravity (m/s^2)

%% Trigonometric Shorthands
c1 = cos(q1); 
c2 = cos(q2); s2 = sin(q2);
c3 = cos(q3); s3 = sin(q3);
c12 = cos(q1 + q2);
c23 = cos(q2 + q3); s23 = sin(q2 + q3);
c123 = cos(q1 + q2 + q3);

%% Dynamic Model Equations from Lagrange Formulation

% 1. Mass Matrix M(q)
M11 = I1 + I2 + I3 + m1*r1^2 + m2*(l1^2 + r2^2 + 2*l1*r2*c2) + m3*(l1^2 + l2^2 + r3^2 + 2*l1*l2*c2 + 2*l1*r3*c23 + 2*l2*r3*c3);
M12 = I2 + I3 + m2*(r2^2 + l1*r2*c2) + m3*(l2^2 + r3^2 + l1*l2*c2 + l1*r3*c23 + 2*l2*r3*c3);
M13 = I3 + m3*(r3^2 + l1*r3*c23 + l2*r3*c3);
M22 = I2 + I3 + m2*r2^2 + m3*(l2^2 + r3^2 + 2*l2*r3*c3);
M23 = I3 + m3*(r3^2 + l2*r3*c3);
M33 = I3 + m3*r3^2;

M = [M11, M12, M13;
     M12, M22, M23;
     M13, M23, M33];

% 2. Gravity Vector G(q)
G1 = g * (m1*r1*c1 + m2*(l1*c1 + r2*c12) + m3*(l1*c1 + l2*c12 + r3*c123));
G2 = g * (m2*r2*c12 + m3*(l2*c12 + r3*c123));
G3 = g * m3*r3*c123;
G = [G1; G2; G3];

% 3. Coriolis and Centrifugal Vector V(q, q_dot)
V1 = -m2*l1*r2*s2*q2_dot^2 - m3*l1*l2*s2*q2_dot^2 - m3*l1*r3*s23*(q2_dot+q3_dot)^2 ...
     - (2*m2*l1*r2*s2 + 2*m3*l1*l2*s2 + 2*m3*l1*r3*s23)*q1_dot*q2_dot ...
     - (2*m3*l1*r3*s23 + 2*m3*l2*r3*s3)*q1_dot*q3_dot ...
     - m3*l2*r3*s3*q3_dot^2 - 2*m3*l2*r3*s3*q2_dot*q3_dot;
     
V2 = (m2*l1*r2*s2 + m3*l1*l2*s2 + m3*l1*r3*s23)*q1_dot^2 ...
     - m3*l2*r3*s3*q3_dot^2 - 2*m3*l2*r3*s3*q1_dot*q3_dot - 2*m3*l2*r3*s3*q2_dot*q3_dot;

V3 = (m3*l1*r3*s23 + m3*l2*r3*s3)*q1_dot^2 + m3*l2*r3*s3*(q2_dot^2 + 2*q1_dot*q2_dot);

V = [V1; V2; V3];

%% Computed Torque Control Law

% Controller gains (proportional and derivative)
Kp = diag([1500, 1200, 900]);
Kd = diag([50, 40, 30]);

% Tracking errors
e = [qd1 - q1; qd2 - q2; qd3 - q3]; % Position error
e_dot = [qd1_dot - q1_dot; qd2_dot - q2_dot; qd3_dot - q3_dot]; % Velocity error

% Desired joint acceleration (control input)
qdd_desired = [qd1_dotdot; qd2_dotdot; qd3_dotdot] + Kd*e_dot + Kp*e;

% Final Torque Calculation
% Torque = M(q) * (desired_acceleration) + V(q,q_dot) + G(q)
Torque_vector = M * qdd_desired + V + G;

%% Output Torques
Torque1 = Torque_vector(1);
Torque2 = Torque_vector(2);
Torque3 = Torque_vector(3);

end
