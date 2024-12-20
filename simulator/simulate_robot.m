function simulate_robot()
    %% Define Fixed Parameters
    %left leg is blue and right leg is red
    % Masses and Inertias for left leg, right leg, and body
    m1 = 0.2393; m2 = 0.0368; m3 = 0.00783; m4 = 0.0155; 
    m5 = 0.2393; m6 = 0.0368; m7 = 0.00783; m8 = 0.0155; % Right leg (same as left leg)
    m_body = 0.01;
    I1 = 25.1e-6; I2 = 53.5e-6; I3 = 9.25e-6; I4 = 22.176e-6; 
    I5 = 25.1e-6; I6 = 53.5e-6; I7 = 9.25e-6; I8 = 22.176e-6; % Right leg
    I_body = 25.1e-6;
    
    % Link lengths and distances from joints to centers of mass
    l_OA = 0.011; l_OB = 0.042; l_AC = 0.096; l_DE = 0.091; l_body = 0.25;
    l_O_m1 = 0.032; l_B_m2 = 0.0344; l_A_m3 = 0.0622; l_C_m4 = 0.0610;
    l_B_m_body = l_body / 2;

    % Motor and gravity parameters
    N = 18.75; Ir = 0.0035 / N^2; g = 9.8;

    % Updated parameter vector to match all defined parameters in derive_robot.m
    p = [m1 m2 m3 m4 m5 m6 m7 m8 m_body I1 I2 I3 I4 I5 I6 I7 I8 I_body Ir N ...
         l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body l_OA l_OB l_AC l_DE l_body g]';

    %% Initial Conditions
    x_base = 3; y_base = .15; th1 = -pi/4; th2 = pi/2; th3 = -pi/4; th4 = pi/2; th5 = 0;
    dx_base = -.02; dy_base = 0; dth1 = 0; dth2 = 0; dth3 = 0; dth4 = 0; dth5 = 0;
    initial_angles = [th1; th2; th3; th4];

    %% Simulation Parameters
    dt = 0.001; tf = 10; tspan = 0:dt:tf;
    num_steps = length(tspan);
    z_out = zeros(14, num_steps);
    z_out(:,1) = [x_base; y_base; th1; th2; th3; th4; th5; dx_base; dy_base; dth1; dth2; dth3; dth4; dth5];
    %% Simulation Loop
    for i = 1:num_steps-1
        % Calculate control torques for both legs
        t = tspan(i);
        tau = control_law(t, z_out(:,i), p);
        %tau = [0;0;0;0];
        %tau = control_law(t, z_out(:,i), p, initial_angles);
        % Compute dynamics
        dz = dynamics(t, z_out(:, i), p, tau);
        
        % Update state
        z_out(:, i+1) = z_out(:, i) + dz * dt;
    end

    %% Compute and Plot Energy
    E = zeros(1, num_steps);
    for i = 1:num_steps
        z = z_out(:, i);  % Extract state at each time step
        E(i) = energy_leg(z, p);  % Compute energy at this state
    end

    % Plot the total energy over time
    figure; clf;
    plot(tspan, E, 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Total Energy (J)');
    title('Total Energy of the Robot Over Time');
    grid on;

    %% Animate the Robot
    animate_robot(tspan, z_out, p);
end

function dz = dynamics(t, z, p, tau)
    q = z(1:7);
    dq = z(8:14);

    % Extract control torques for each leg
    tau_left = tau(1:2);
    tau_right = tau(3:4);

    % Define external forces on left and right feet (horizontal and vertical components)
    [Fc_left, Fc_right] = contact_forces(z, p);

    % Get Jacobians for each foot
    J_left = jacobian_left_foot(z, p);
    J_right = jacobian_right_foot(z, p);

    % Compute generalized forces from contact forces
    QFc_left = J_left' * Fc_left;
    QFc_right = J_right' * Fc_right;

    % Total generalized force due to contact forces
    QFc = QFc_left + QFc_right;

    % Compute mass matrix and dynamics vector
    A = A_leg(z, p);
    b = b_leg(z, [tau_left; tau_right], Fc_left, Fc_right, p);

    % Compute accelerations
    ddq = A \ (b + QFc);

    % Form dz (state derivative)
    dz = [dq; ddq];
end

function [Fc_left, Fc_right] = contact_forces(z, p)
    % This function computes both horizontal and vertical contact forces for the left and right feet.
    % Fc_left and Fc_right are now vectors with both horizontal (x) and vertical (y) components.

    % Retrieve positions and velocities of the feet
    rE_left = position_left_foot(z, p);
    rE_right = position_right_foot(z, p);
    drE_left = velocity_left_foot(z, p);
    drE_right = velocity_right_foot(z, p);

    % Parameters for ground contact
    ground_level = 0;
    Kc = 1000;  % Spring constant for contact
    Dc = 20;    % Damping coefficient for contact
    mu = 1.5;   % Friction coefficient

    % Initialize contact forces (horizontal and vertical components)
    Fc_left = [0; 0];
    Fc_right = [0; 0];

    % Right foot contact calculation
    if rE_right(2) <= ground_level
        % Vertical contact force
        delta_right = ground_level - rE_right(2);       % Penetration depth
        delta_dot_right = -drE_right(2);                % Relative vertical velocity
        Fy_right = max(Kc * delta_right + Dc * delta_dot_right, 0);

        % Horizontal contact force (friction)
        Vx_right = drE_right(1); % Horizontal velocity of the right foot
        if abs(Vx_right) > 1e-3  % Sliding condition
            Fx_right = -sign(Vx_right) * mu * Fy_right;
        else                     % Static friction condition (no sliding)
            Fx_right = -min(mu * Fy_right, abs(Kc * Vx_right)) * sign(Vx_right);
        end
        %Fx_right = 0;

        % Combine into total contact force for right foot
        Fc_right = [Fx_right; Fy_right];
    end

    % Left foot contact calculation
    if rE_left(2) <= ground_level
        % Vertical contact force
        delta_left = ground_level - rE_left(2);         % Penetration depth
        delta_dot_left = -drE_left(2);                  % Relative vertical velocity
        Fy_left = max(Kc * delta_left + Dc * delta_dot_left, 0);

        % Horizontal contact force (friction)
        Vx_left = drE_left(1); % Horizontal velocity of the left foot
        if abs(Vx_left) > 1e-3  % Sliding condition
            Fx_left = -sign(Vx_left) * mu * Fy_left;
        else                     % Static friction condition (no sliding)
            Fx_left = -min(mu * Fy_left, abs(Kc * Vx_left)) * sign(Vx_left);
        end
        %Fx_left = 0;

        % Combine into total contact force for left foot
        Fc_left = [Fx_left; Fy_left];
    end
end
function tau = control_law(t, z, p)
    % Define torsional spring and damper parameters
    K_torsional = 4;    % Torsional spring constant
    D_torsional = 0.2;  % Damping constant
    max_torque = 5;     % Max torque limit

    % Extract joint angles and velocities
    th1 = z(3);   % Left hip joint
    th2 = z(4);   % Left knee joint
    th3 = z(5);   % Right hip joint
    th4 = z(6);   % Right knee joint
    dth1 = z(10); % Left hip joint velocity
    dth2 = z(11); % Left knee joint velocity
    dth3 = z(12); % Right hip joint velocity
    dth4 = z(13); % Right knee joint velocity

    % Elliptical trajectory parameters in joint space
    omega_h = 15;                % Angular frequency  for hip % -15
    omega_k = 15;                 % Ang freq for knee
    hip_amp = pi / 8;         % Amplitude of hip oscillation (controls side-to-side range) pi / 8;
    knee_amp = pi / 12 ;       % Amplitude of knee oscillation (controls up-down range) pi / 12;
    phase_offset = pi;        % Phase offset for the right leg

    % Desired joint angles for elliptical path
    th1_des = -pi/4 + hip_amp * cos(omega_h * t);             % Left hip follows a cosine wave around initial angle
    th2_des = pi/2 + knee_amp * sin(omega_k * t);             % Left knee follows a sine wave around initial angle
    th3_des = -pi/4 + hip_amp * cos(omega_h * t + phase_offset); % Right hip, 180 degrees out of phase
    th4_des = pi/2 + knee_amp * sin(omega_k * t + phase_offset); % Right knee, 180 degrees out of phase

    % Virtual torsional spring-damper torques for each joint
    tau1 = -K_torsional * (th1 - th1_des) - D_torsional * dth1;  % Left hip
    tau2 = -K_torsional * (th2 - th2_des) - D_torsional * dth2;  % Left knee
    tau3 = -K_torsional * (th3 - th3_des) - D_torsional * dth3;  % Right hip
    tau4 = -K_torsional * (th4 - th4_des) - D_torsional * dth4;  % Right knee

    % Clamp torques to prevent excessive values
    tau1 = max(-max_torque, min(tau1, max_torque));
    tau2 = max(-max_torque, min(tau2, max_torque));
    tau3 = max(-max_torque, min(tau3, max_torque));
    tau4 = max(-max_torque, min(tau4, max_torque));

    % Combine torques into a single vector
    tau = [tau1; tau2; tau3; tau4];
end
function animate_robot(tspan, z_out, p)
    figure(1); clf; hold on;
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    title('Robot Animation');
    axis equal; axis([-3.5 3.5 -0.5 1.5]);
    plot([-5, 5], [0, 0], 'k--', 'LineWidth', 2);

    % Initialize plot handles for the body, left leg, and right leg
    h_body = plot([0, 0], [0, 0], 'k', 'LineWidth', 4);
    h_left_leg = plot([0, 0, 0, 0], [0, 0, 0, 0], 'b', 'LineWidth', 2);
    h_right_leg = plot([0, 0, 0, 0], [0, 0, 0, 0], 'r', 'LineWidth', 2);

    for i = 1:10:length(tspan)
        z = z_out(:, i);

        % Get keypoints for visualization
        keypoints = keypoints_leg(z, p);

        % Extract key points for the left leg, right leg, and body
        % Left leg keypoints
        rO = keypoints(:, 11);       % Left hip (O)
        rA = keypoints(:, 1);        % Left thigh joint (A)
        rC = keypoints(:, 3);        % Left knee joint (C)
        rE_left = keypoints(:, 5);   % Left foot (E)

        % Right leg keypoints
        rOR = keypoints(:, 12);      % Right hip (OR)
        rA_right = keypoints(:, 6);  % Right thigh joint (AR)
        rC_right = keypoints(:, 8);  % Right knee joint (CR)
        rE_right = keypoints(:, 10); % Right foot (ER)

        % Define body linkage (from left hip to right hip)
        set(h_body, 'XData', [rO(1), rOR(1)], 'YData', [rO(2), rOR(2)]);

        % Define left leg linkage coordinates
        left_leg_x = [rO(1), rA(1), rC(1), rE_left(1)];
        left_leg_y = [rO(2), rA(2), rC(2), rE_left(2)];
        
        % Define right leg linkage coordinates
        right_leg_x = [rOR(1), rA_right(1), rC_right(1), rE_right(1)];
        right_leg_y = [rOR(2), rA_right(2), rC_right(2), rE_right(2)];

        % Update left leg plot
        set(h_left_leg, 'XData', left_leg_x, 'YData', left_leg_y);

        % Update right leg plot
        set(h_right_leg, 'XData', right_leg_x, 'YData', right_leg_y);

        drawnow;

        % Pause for a moment
        pause(0.01);
    end
end
