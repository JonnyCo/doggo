clear
name = 'leg';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t x_base y_base th1 th2 th3 th4 th5 dx_base dy_base dth1 dth2 dth3 dth4 dth5 ...
     ddx_base ddy_base ddth1 ddth2 ddth3 ddth4 ddth5 real

% Define masses and inertias for each link (legs and body)
syms m1 m2 m3 m4 m_body I1 I2 I3 I4 I_body real

% Define lengths from each joint to the center of mass of each link
syms l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body real

% Define link lengths
syms l_OA l_OB l_AC l_DE l_body real

% Define gravitational constant
syms g real

% Define control torques for each joint (hip and knee joints for both legs)
syms tau1 tau2 tau3 tau4 real

% Define motor parameters, if applicable (e.g., gear ratio and rotor inertia)
syms Ir N real

% Group generalized coordinates, first and second derivatives
q   = [x_base; y_base; th1; th2; th3; th4; th5];
dq  = [dx_base; dy_base; dth1; dth2; dth3; dth4; dth5];
ddq = [ddx_base; ddy_base; ddth1; ddth2; ddth3; ddth4; ddth5];

% Control vector only for actuated joints
u   = [tau1; tau2; tau3; tau4];

% Group all parameters in a parameter vector
p = [m1 m2 m3 m4 m_body I1 I2 I3 I4 I_body Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body ...
     l_OA l_OB l_AC l_DE l_body g]';

% Generate Vectors and Derivatives
ihat = [1; 0; 0];  % x-direction
jhat = [0; -1; 0]; % y-direction with gravity downward
khat = [0; 0; 1];  % z-direction

% Unit vectors along the directions defined by each joint angle
e1hat = cos(th1) * ihat + sin(th1) * jhat;
e2hat = cos(th1 + th2) * ihat + sin(th1 + th2) * jhat;
e3hat = cos(th3) * ihat + sin(th3) * jhat;
e4hat = cos(th3 + th4) * ihat + sin(th3 + th4) * jhat;
e5hat = cos(th5) * ihat + sin(th5) * jhat;  % Body orientation

% Define an anonymous function for taking time derivatives
ddt = @(r) jacobian(r, [q; dq]) * [dq; ddq];

% Base position vector
r_base = [x_base; y_base; 0];

% Position vectors for points on the left leg
rA = r_base + l_OA * e1hat;
rB = r_base + l_OB * e1hat;
rC = rA + l_AC * e2hat;
rD = rB + l_AC * e2hat;
rE_left = rD + l_DE * e2hat;

% Position vectors for points on the right leg
rM = r_base + l_body * e5hat;     % Position of body end point
rA_right = rM + l_OA * e3hat;
rB_right = rM + l_OB * e3hat;
rC_right = rA_right + l_AC * e4hat;
rD_right = rB_right + l_AC * e4hat;
rE_right = rD_right + l_DE * e4hat;

% Center of mass positions for each link
r_m1 = r_base + l_O_m1 * e1hat;           % Left thigh CoM
r_m2 = rB + l_B_m2 * e2hat;               % Left shank CoM
r_m3 = rA_right + l_A_m3 * e4hat;         % Right thigh CoM
r_m4 = rC_right + l_C_m4 * e3hat;         % Right shank CoM
r_m_body = r_base + l_B_m_body * e5hat;   % Body CoM

% Compute derivatives of position vectors
drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE_left = ddt(rE_left);
drA_right = ddt(rA_right);
drB_right = ddt(rB_right);
drC_right = ddt(rC_right);
drD_right = ddt(rD_right);
drE_right = ddt(rE_right);

% Derivatives for center of mass positions
dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);
dr_m3 = ddt(r_m3);
dr_m4 = ddt(r_m4);
dr_m_body = ddt(r_m_body);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F, r) simplify(jacobian(r, q).' * F);    % Force contributions to generalized forces
M2Q = @(M, w) simplify(jacobian(w, dq).' * M);   % Moment contributions to generalized forces

% Define angular velocities for each segment
omega1 = dth1;
omega2 = dth1 + dth2;
omega3 = dth3;
omega4 = dth3 + dth4;
omega_body = dth5;

% Kinetic energy for each segment
T1 = (1/2) * m1 * dot(dr_m1, dr_m1) + (1/2) * I1 * omega1^2;
T2 = (1/2) * m2 * dot(dr_m2, dr_m2) + (1/2) * I2 * omega2^2;
T3 = (1/2) * m3 * dot(dr_m3, dr_m3) + (1/2) * I3 * omega3^2;
T4 = (1/2) * m4 * dot(dr_m4, dr_m4) + (1/2) * I4 * omega4^2;
T_body = (1/2) * m_body * dot(dr_m_body, dr_m_body) + (1/2) * I_body * omega_body^2;

% Add terms for motor/rotor inertias, if applicable
T1r = (1/2) * Ir * (N * dth1)^2;
T2r = (1/2) * Ir * (N * dth2)^2;

% Total Kinetic Energy
T = simplify(T1 + T2 + T3 + T4 + T_body + T1r + T2r);

% Potential energy for each segment
Vg1 = m1 * g * r_m1(2);
Vg2 = m2 * g * r_m2(2);
Vg3 = m3 * g * r_m3(2);
Vg4 = m4 * g * r_m4(2);
Vg_body = m_body * g * r_m_body(2);

% Total Potential Energy
Vg = Vg1 + Vg2 + Vg3 + Vg4 + Vg_body;

% Generalized forces due to control torques for actuated joints
Q_tau1 = M2Q(tau1 * khat, omega1 * khat);
Q_tau2 = M2Q(tau2 * khat, omega2 * khat);
Q_tau3 = M2Q(tau3 * khat, omega3 * khat);
Q_tau4 = M2Q(tau4 * khat, omega4 * khat);

% Sum of generalized forces (no control torque on th5)
Q_tau = Q_tau1 + Q_tau2 + Q_tau3 + Q_tau4;

% External forces acting on the body's center of mass (if any)
F_body_ext = [0; 0; 0];  % Placeholder for external force on body CoM
Q_body_ext = F2Q(F_body_ext, r_m_body);  % Generalized forces due to external force on body

% Total Generalized Force Vector
Q = Q_tau + Q_body_ext;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE_left(1:2) ...
             rA_right(1:2) rB_right(1:2) rC_right(1:2) rE_right(1:2) ...
             rM(1:2)]; % Body endpoint for visualization

%% All the work is done! Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T + Vg;           % Total energy
L = T - Vg;           % Lagrangian
eom = ddt(jacobian(L, dq).') - jacobian(L, q).' - Q;  % Equations of motion

% Rearrange Equations of Motion
A = simplify(jacobian(eom, ddq));    % Mass matrix
n = simplify(eom - A * ddq);         % Non-inertial terms (Coriolis, gravity, etc.)
b = -n;                              % Right-hand side of equations of motion

% Equations of motion are:
% A * ddq + n = 0
% So we have: A * ddq = -n

% Compute foot Jacobians for both feet
J_left = jacobian(rE_left(1:2), q);    % Left foot Jacobian (2 x 7)
J_right = jacobian(rE_right(1:2), q);  % Right foot Jacobian (2 x 7)

% Compute ddt( J ) for both feet
dJ_left = reshape(ddt(J_left(:)), size(J_left));
dJ_right = reshape(ddt(J_right(:)), size(J_right));

% Compute Jacobian for the body's center of mass
J_body = jacobian(r_m_body(1:2), q);    % Body CoM Jacobian (2 x 7)
