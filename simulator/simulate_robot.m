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
    l_OA = 0.011; l_OB = 0.042; l_AC = 0.096; l_DE = 0.091; l_body = 0.5;
    l_O_m1 = 0.032; l_B_m2 = 0.0344; l_A_m3 = 0.0622; l_C_m4 = 0.0610;
    l_B_m_body = l_body / 2;

    % Motor and gravity parameters
    N = 18.75; Ir = 0.0035 / N^2; g = 3;

    % Updated parameter vector to match all defined parameters in derive_robot.m
    p = [m1 m2 m3 m4 m5 m6 m7 m8 m_body I1 I2 I3 I4 I5 I6 I7 I8 I_body Ir N ...
         l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body l_OA l_OB l_AC l_DE l_body g]';

    %% Initial Conditions
    x_base = 0; y_base = .2; th1 = -pi/4; th2 = pi/2; th3 = -pi/4; th4 = pi/2; th5 = 0;
    dx_base = 0; dy_base = 0; dth1 = 0; dth2 = 0; dth3 = 0; dth4 = 0; dth5 = 0;
    
    % Define trajectory parameters for left and right legs
    p_traj_left.omega = .0003;
    p_traj_left.x_0   = -0.02;   % Center of ellipse for the left foot, relative to body
    p_traj_left.y_0   = -0.125;
    p_traj_left.r     = 0.025;   % Radius of ellipse for the left foot

    p_traj_right.omega = -.0003;
    p_traj_right.x_0   = 0.02;   % Center of ellipse for the right foot, relative to body
    p_traj_right.y_0   = -0.125;
    p_traj_right.r     = 0.025;  % Radius of ellipse for the right foot

    %% Simulation Parameters
    dt = 0.001; tf = 1; tspan = 0:dt:tf;
    num_steps = length(tspan);
    z_out = zeros(14, num_steps);
    z_out(:,1) = [x_base; y_base; th1; th2; th3; th4; th5; dx_base; dy_base; dth1; dth2; dth3; dth4; dth5];
    %% Simulation Loop
    for i = 1:num_steps-1
        % Calculate control torques for both legs
        t = tspan(i);
        %tau = control_law(t, z_out(:,i), p, p_traj_left, p_traj_right);
        tau = [0;0;0;0];
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
    mu = 0.8;   % Friction coefficient

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

        % Combine into total contact force for left foot
        Fc_left = [Fx_left; Fy_left];
    end
end



function tau = control_law(t, z, p, p_traj_left, p_traj_right)
    % Controller gains for both legs
    K_x = 150; % Spring stiffness X
    K_y = 150; % Spring stiffness Y
    D_x = 10;  % Damping X
    D_y = 10;  % Damping Y

    % Define stiffness and damping matrices
    K = diag([K_x, K_y]);
    D = diag([D_x, D_y]);

    %% Left Leg Control
    % Desired position, velocity, and acceleration of the left foot
    omega_swing_left = p_traj_left.omega;
    rEd_left = [p_traj_left.x_0; p_traj_left.y_0; 0] + ...
               p_traj_left.r * [cos(omega_swing_left * t); sin(omega_swing_left * t); 0];
    vEd_left = p_traj_left.r * [-sin(omega_swing_left * t) * omega_swing_left; ...
                                 cos(omega_swing_left * t) * omega_swing_left; 0];
    aEd_left = p_traj_left.r * [-cos(omega_swing_left * t) * omega_swing_left^2; ...
                                -sin(omega_swing_left * t) * omega_swing_left^2; 0];

    % Actual position and velocity of the left foot
    rE_left = position_left_foot(z, p);
    vE_left = velocity_left_foot(z, p);

    % Compute virtual force for left leg
    f_left = [K_x * (rEd_left(1) - rE_left(1)) + D_x * (vEd_left(1) - vE_left(1)); ...
              K_y * (rEd_left(2) - rE_left(2)) + D_y * (vEd_left(2) - vE_left(2))];

    % Map to joint torques for left leg
    J_left = jacobian_left_foot(z, p);
    M_left = A_leg(z, p);
    lambda_left = inv(J_left * inv(M_left) * J_left');
    mu_left = Corr_leg(z, p);
    sigma_left = Grav_leg(z, p);
    f_left = [lambda_left(1,1)*(aEd_left(1) + K_x * (rEd_left(1) - rE_left(1)) + D_x * (vEd_left(1) - vE_left(1))) + mu_left(1) + sigma_left(1);
              lambda_left(2,2)*(aEd_left(2) + K_y * (rEd_left(2) - rE_left(2)) + D_y * (vEd_left(2) - vE_left(2))) + mu_left(2) + sigma_left(2)];
    tau_left = J_left' * f_left;

    %% Right Leg Control
    % Desired position, velocity, and acceleration of the right foot
    omega_swing_right = p_traj_right.omega;
    rEd_right = [p_traj_right.x_0; p_traj_right.y_0; 0] + ...
                p_traj_right.r * [cos(omega_swing_right * t); sin(omega_swing_right * t); 0];
    vEd_right = p_traj_right.r * [-sin(omega_swing_right * t) * omega_swing_right; ...
                                   cos(omega_swing_right * t) * omega_swing_right; 0];
    aEd_right = p_traj_right.r * [-cos(omega_swing_right * t) * omega_swing_right^2; ...
                                  -sin(omega_swing_right * t) * omega_swing_right^2; 0];

    % Actual position and velocity of the right foot
    rE_right = position_right_foot(z, p);
    vE_right = velocity_right_foot(z, p);

    % Compute virtual force for right leg
    f_right = [K_x * (rEd_right(1) - rE_right(1)) + D_x * (vEd_right(1) - vE_right(1)); ...
               K_y * (rEd_right(2) - rE_right(2)) + D_y * (vEd_right(2) - vE_right(2))];

    % Map to joint torques for right leg
    J_right = jacobian_right_foot(z, p);
    M_right = A_leg(z, p);
    lambda_right = inv(J_right * inv(M_right) * J_right');
    mu_right = Corr_leg(z, p);
    sigma_right = Grav_leg(z, p);
    f_right = [lambda_right(1,1)*(aEd_right(1) + K_x * (rEd_right(1) - rE_right(1)) + D_x * (vEd_right(1) - vE_right(1))) + mu_right(1) + sigma_right(1);
               lambda_right(2,2)*(aEd_right(2) + K_y * (rEd_right(2) - rE_right(2)) + D_y * (vEd_right(2) - vE_right(2))) + mu_right(2) + sigma_right(2)];
    tau_right = J_right' * f_right;

    %% Combine Torques
    % Concatenate torques for both legs
    tau = [tau_left; tau_right];
end


function animate_robot(tspan, z_out, p)
    figure(1); clf; hold on;
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    title('Robot Dropping Simulation');
    axis equal; axis([-1 1 -0.5 1.5]);
    plot([-5, 5], [0, 0], 'k--', 'LineWidth', 2);

    % Initialize plot handles for the body, left leg, and right leg
    h_body = plot([0, 0], [0, 0], 'k', 'LineWidth', 4);
    h_left_leg = plot([0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 'b', 'LineWidth', 2);
    h_right_leg = plot([0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 'r', 'LineWidth', 2);

    for i = 1:10:length(tspan)
        z = z_out(:, i);
        x_base = z(1);
        y_base = z(2);
        r_base = [x_base; y_base];

        % Get keypoints for visualization
        keypoints = keypoints_leg(z, p);

        % Extract key points for the left leg, right leg, and body
        % Left leg keypoints
        rO = keypoints(:, 11);       % Left hip (O)
        rA = keypoints(:, 1);        % Left thigh joint (A)
        rB = keypoints(:, 2);        % Left thigh center (B)
        rC = keypoints(:, 3);        % Left knee joint (C)
        rD = keypoints(:, 4);        % Left shank center (D)
        rE_left = keypoints(:, 5);   % Left foot (E)

        % Right leg keypoints
        rOR = keypoints(:, 12);      % Right hip (OR)
        rA_right = keypoints(:, 6);  % Right thigh joint (AR)
        rB_right = keypoints(:, 7);  % Right thigh center (BR)
        rC_right = keypoints(:, 8);  % Right knee joint (CR)
        rD_right = keypoints(:, 9);  % Right shank center (DR)
        rE_right = keypoints(:, 10); % Right foot (ER)

        % Define body linkage (from left hip to right hip)
        set(h_body, 'XData', [rO(1), rOR(1)], 'YData', [rO(2), rOR(2)]);

        % Define left leg linkage coordinates
        left_leg_x = [rO(1), rA(1), rC(1), rE_left(1)];
        left_leg_y = [rO(2), rA(2), rC(2), rE_left(2)];
        left_leg_x2 = [rO(1), rB(1), rD(1), rE_left(1)];
        left_leg_y2 = [rO(2), rB(2), rD(2), rE_left(2)];

        % Define right leg linkage coordinates
        right_leg_x = [rOR(1), rA_right(1), rC_right(1), rE_right(1)];
        right_leg_y = [rOR(2), rA_right(2), rC_right(2), rE_right(2)];
        right_leg_x2 = [rOR(1), rB_right(1), rD_right(1), rE_right(1)];
        right_leg_y2 = [rOR(2), rB_right(2), rD_right(2), rE_right(2)];

        % Update left leg plot (primary linkage)
        set(h_left_leg, 'XData', left_leg_x, 'YData', left_leg_y);

        % Update left leg plot (secondary linkage)
        h_left_leg2 = plot(left_leg_x2, left_leg_y2, 'b--', 'LineWidth', 1);

        % Update right leg plot (primary linkage)
        set(h_right_leg, 'XData', right_leg_x, 'YData', right_leg_y);

        % Update right leg plot (secondary linkage)
        h_right_leg2 = plot(right_leg_x2, right_leg_y2, 'r--', 'LineWidth', 1);

        % Plot markers for each keypoint (for debugging and visualization)
        plot(rO(1), rO(2), 'ro');         % Left hip joint (O)
        plot(rA(1), rA(2), 'go');         % Left thigh joint (A)
        plot(rC(1), rC(2), 'co');         % Left knee joint (C)
        plot(rE_left(1), rE_left(2), 'bo'); % Left foot endpoint (E)
        
        plot(rOR(1), rOR(2), 'rx');       % Right hip joint (OR)
        plot(rA_right(1), rA_right(2), 'gx'); % Right thigh joint (AR)
        plot(rC_right(1), rC_right(2), 'bx'); % Right knee joint (CR)
        plot(rE_right(1), rE_right(2), 'kx'); % Right foot endpoint (ER)

        drawnow;

        % Delete secondary linkage plots to avoid accumulation
        %delete(h_left_leg2);
        %delete(h_right_leg2);

        pause(0.01);
    end
end

