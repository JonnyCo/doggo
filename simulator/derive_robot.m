clear
name = 'leg';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t x_base y_base th1 th2 th3 th4 th5 dx_base dy_base dth1 dth2 dth3 dth4 dth5 ...
     ddx_base ddy_base ddth1 ddth2 ddth3 ddth4 ddth5 real

% Define masses and inertias for each link (left leg, right leg, and body)
syms m1 m2 m3 m4 m5 m6 m7 m8 m_body I1 I2 I3 I4 I5 I6 I7 I8 I_body real

% Define lengths from each joint to the center of mass of each link
syms l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body real

% Define link lengths
syms l_OA l_OB l_AC l_DE l_body real

% Define gravitational constant
syms g real

% Define control torques for each joint (hip and knee joints for both legs)
syms tau1 tau2 tau3 tau4 real

syms Fx1 Fy1 Fx2 Fy2 real  % Forces acting on the left and right legs, respectively

% Define motor parameters, if applicable (e.g., gear ratio and rotor inertia)
syms Ir N real

% Group generalized coordinates, first and second derivatives
q   = [x_base; y_base; th1; th2; th3; th4; th5];
dq  = [dx_base; dy_base; dth1; dth2; dth3; dth4; dth5];
ddq = [ddx_base; ddy_base; ddth1; ddth2; ddth3; ddth4; ddth5];

% Control vector only for actuated joints
u   = [tau1; tau2; tau3; tau4];

% External cartesian forces
F_left = [Fx1; Fy1];  % 2D forces for left foot
F_right = [Fx2; Fy2]; % 2D forces for right foot

% Group all parameters in a parameter vector
p = [m1 m2 m3 m4 m5 m6 m7 m8 m_body I1 I2 I3 I4 I5 I6 I7 I8 I_body Ir N l_O_m1 l_B_m2 ...
     l_A_m3 l_C_m4 l_B_m_body l_OA l_OB l_AC l_DE l_body g]';

ihat = [1; 0; 0];      % Points rightward in the global frame (positive x-axis)
jhat = [0; 1; 0];      % Points upward in the global frame (positive y-axis)
khat = cross(ihat, jhat);  % Points out of the plane (z-axis), for moments


% Define unit vectors along the directions defined by each joint angle
% The joint angles th1, th2, etc., are applied to each link in sequence from the base.

% Left leg. okay i hate that i subtracted pi/2 just to make the legs in the
% right direction...solve later
% Position vectors for points on the left leg relative to the left hip (O)

e1hat = cos(th1 + th5 - pi/2) * ihat + sin(th1 + th5 - pi/2) * jhat;
e2hat = cos(th1 + th2 + th5 - pi/2) * ihat + sin(th1 + th2 + th5 - pi/2) * jhat;
e3hat = cos(th3 + th5 - pi/2) * ihat + sin(th3 + th5 - pi/2) * jhat;
e4hat = cos(th3 + th4 + th5 - pi/2) * ihat + sin(th3 + th4 + th5 - pi/2) * jhat;
% Body orientation
e5hat = cos(th5) * ihat + sin(th5) * jhat;               % Body link from O to OR


% Define the base position at the midpoint between the left and right hips
r_base = [x_base; y_base; 0];  % New origin, centered between hips

% Define left and right hip joints relative to the centered base
rO = r_base - (l_body / 2) * e5hat;   % Left hip joint (O)
rOR = r_base + (l_body / 2) * e5hat;  % Right hip joint (OR)

% Position vectors for points on the left leg relative to the left hip (O)
rA = rO + l_OA * e1hat;               % Left hip joint (A)
rB = rO + l_OB * e1hat;               % Left thigh joint (B)
rC = rA + l_AC * e2hat;               % Left knee (C)
rD = rB + l_AC * e2hat;               % Left shank joint (D)
rE = rD + l_DE * e1hat;               % Left foot endpoint (E)

% Position vectors for points on the right leg relative to the right hip (OR)
rA_right = rOR + l_OA * e3hat;        % Right hip joint (AR)
rB_right = rOR + l_OB * e3hat;        % Right thigh joint (BR)
rC_right = rA_right + l_AC * e4hat;   % Right knee (CR)
rD_right = rB_right + l_AC * e4hat;   % Right shank joint (DR)
rE_right = rD_right + l_DE * e3hat;   % Right foot endpoint (ER)

% Center of mass positions for each link

% Left leg
r_m1 = rO + l_O_m1 * e1hat;             % Left thigh CoM
r_m2 = rB + l_B_m2 * e2hat;             % Left shank CoM
r_m3 = rA + l_A_m3 * e2hat;             % Left knee link CoM
r_m4 = rC + l_C_m4 * e1hat;             % Left foot link CoM

% Right leg
r_m5 = rOR + l_O_m1 * e3hat;            % Right thigh CoM
r_m6 = rB_right + l_B_m2 * e4hat;       % Right shank CoM
r_m7 = rA_right + l_A_m3 * e4hat;       % Right knee link CoM
r_m8 = rC_right + l_C_m4 * e3hat;       % Right foot link CoM

r_m_body = r_base;                        % Body CoM (since it's centered)


% Define an anonymous function for taking time derivatives
ddt = @(r) jacobian(r, [q; dq]) * [dq; ddq];

% Compute derivatives of position vectors
drA = ddt(rA);
drB = ddt(rB);
drC = ddt(rC);
drD = ddt(rD);
drE = ddt(rE);
drA_right = ddt(rA_right);
drB_right = ddt(rB_right);
drC_right = ddt(rC_right);
drD_right = ddt(rD_right);
drE_right = ddt(rE_right);

% Derivatives for center of mass positions for each link
dr_m1 = ddt(r_m1);             % Left thigh CoM
dr_m2 = ddt(r_m2);             % Left shank CoM
dr_m3 = ddt(r_m3);             % Left knee link CoM
dr_m4 = ddt(r_m4);             % Left foot link CoM
dr_m5 = ddt(r_m5);             % Right thigh CoM
dr_m6 = ddt(r_m6);             % Right shank CoM
dr_m7 = ddt(r_m7);             % Right knee link CoM
dr_m8 = ddt(r_m8);             % Right foot link CoM
dr_m_body = ddt(r_m_body);     % Body CoM

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q_2D = @(F, r) simplify(jacobian(r, q(1:2)).' * F);    % Force contributions to generalized forces in 2D
M2Q = @(M, w) simplify(jacobian(w, dq).' * M);   % Moment contributions to generalized forces

% Define angular velocities for each segment
omega1 = dth1;
omega2 = dth1 + dth2;
omega3 = dth3;
omega4 = dth3 + dth4;
omega_body = dth5;

omega1_total = dth5 + dth1;             % Left hip relative to the body’s rotation
omega2_total = dth5 + dth1 + dth2;      % Left knee relative to the body’s rotation

omega3_total = dth5 + dth3;             % Right hip relative to the body’s rotation
omega4_total = dth5 + dth3 + dth4;      % Right knee relative to the body’s rotation

% Kinetic energy for each segment with updated angular velocities
T1 = (1/2) * m1 * dot(dr_m1, dr_m1) + (1/2) * I1 * omega1_total^2;
T2 = (1/2) * m2 * dot(dr_m2, dr_m2) + (1/2) * I2 * omega2_total^2;
T3 = (1/2) * m3 * dot(dr_m3, dr_m3) + (1/2) * I3 * omega2_total^2; % Left knee link CoM
T4 = (1/2) * m4 * dot(dr_m4, dr_m4) + (1/2) * I4 * omega1_total^2; % Left foot link CoM
T5 = (1/2) * m5 * dot(dr_m5, dr_m5) + (1/2) * I1 * omega3_total^2; % Right thigh CoM
T6 = (1/2) * m6 * dot(dr_m6, dr_m6) + (1/2) * I2 * omega4_total^2; % Right shank CoM
T7 = (1/2) * m7 * dot(dr_m7, dr_m7) + (1/2) * I3 * omega4_total^2; % Right knee link CoM
T8 = (1/2) * m8 * dot(dr_m8, dr_m8) + (1/2) * I4 * omega3_total^2; % Right foot link CoM
T_body = (1/2) * m_body * dot(dr_m_body, dr_m_body) + (1/2) * I_body * omega_body^2;

% Add terms for motor/rotor inertias, if applicable
T1r = (1/2) * Ir * (N * dth1)^2;
T2r = (1/2) * Ir * (N * dth2)^2;

% Total Kinetic Energy
T = simplify(T1 + T2 + T3 + T4 + T5 + T6 + T7 + T8 + T_body + T1r + T2r);

% Potential energy for each segment
Vg1 = m1 * g * r_m1(2);
Vg2 = m2 * g * r_m2(2);
Vg3 = m3 * g * r_m3(2);
Vg4 = m4 * g * r_m4(2);
Vg5 = m5 * g * r_m5(2);
Vg6 = m6 * g * r_m6(2);
Vg7 = m7 * g * r_m7(2);
Vg8 = m8 * g * r_m8(2);
Vg_body = m_body * g * r_m_body(2);

% Total Potential Energy
Vg = Vg1 + Vg2 + Vg3 + Vg4 + Vg5 + Vg6 + Vg7 + Vg8 + Vg_body;

% Generalized forces due to control torques for actuated joints
Q_tau1 = M2Q(tau1 * khat, omega1 * khat);
Q_tau2 = M2Q(tau2 * khat, omega2 * khat);
Q_tau3 = M2Q(tau3 * khat, omega3 * khat);
Q_tau4 = M2Q(tau4 * khat, omega4 * khat);

% Generalized forces due to external forces on each foot
Q_F_left = [F2Q_2D(F_left, rE(1:2)); zeros(5, 1)];   % Extend to 7x1 by padding zeros for angles
Q_F_right = [F2Q_2D(F_right, rE_right(1:2)); zeros(5, 1)]; % Extend to 7x1 by padding zeros for angles

% Sum of generalized forces
Q_tau = Q_tau1 + Q_tau2 + Q_tau3 + Q_tau4 + Q_F_left + Q_F_right;

% External forces acting on the body's center of mass (if any)
F_body_ext = [0; 0]; % Assuming no external forces on CoM
Q_body_ext = [F2Q_2D(F_body_ext, r_m_body(1:2)); zeros(5, 1)];

% Total Generalized Force Vector
Q = Q_tau + Q_body_ext;

% Assemble the array of cartesian coordinates of the key points
% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rD(1:2) rE(1:2) ...
             rA_right(1:2) rB_right(1:2) rC_right(1:2) rD_right(1:2) rE_right(1:2) ...
             rO(1:2) rOR(1:2)];  % Include hips for visualization
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
J_left = jacobian(rE(1:2), q);    % Left foot Jacobian (2 x 7)
J_right = jacobian(rE_right(1:2), q);  % Right foot Jacobian (2 x 7)

% Compute ddt( J ) for both feet
dJ_left = reshape(ddt(J_left(:)), size(J_left));
dJ_right = reshape(ddt(J_right(:)), size(J_right));

% Compute Jacobian for the body's center of mass
J_body = jacobian(r_m_body(1:2), q);    % Body CoM Jacobian (2 x 7)
dJ_body = reshape(ddt(J_body(:)), size(J_body));

% Write Energy Function and Equations of Motion
z = [q; dq];  % State vector

% Generate MATLAB functions
matlabFunction(A, 'file', ['A_' name], 'vars', {z, p});
matlabFunction(b, 'file', ['b_' name], 'vars', {z, u, F_left, F_right, p});
matlabFunction(E, 'file', ['energy_' name], 'vars', {z, p});

% Left foot
matlabFunction(rE(1:2), 'file', ['position_left_foot'], 'vars', {z, p});
matlabFunction(ddt(rE(1:2)), 'file', ['velocity_left_foot'], 'vars', {z, p});
matlabFunction(J_left, 'file', ['jacobian_left_foot'], 'vars', {z, p});
matlabFunction(dJ_left, 'file', ['jacobian_dot_left_foot'], 'vars', {z, p});

% Right foot
matlabFunction(rE_right(1:2), 'file', ['position_right_foot'], 'vars', {z, p});
matlabFunction(ddt(rE_right(1:2)), 'file', ['velocity_right_foot'], 'vars', {z, p});
matlabFunction(J_right, 'file', ['jacobian_right_foot'], 'vars', {z, p});
matlabFunction(dJ_right, 'file', ['jacobian_dot_right_foot'], 'vars', {z, p});

% Body center of mass
matlabFunction(r_m_body(1:2), 'file', ['position_body'], 'vars', {z, p});
matlabFunction(ddt(r_m_body(1:2)), 'file', ['velocity_body'], 'vars', {z, p});
matlabFunction(J_body, 'file', ['jacobian_body'], 'vars', {z, p});
matlabFunction(dJ_body, 'file', ['jacobian_dot_body'], 'vars', {z, p});

% Keypoints for visualization
matlabFunction(keypoints, 'file', ['keypoints_' name], 'vars', {z, p});
