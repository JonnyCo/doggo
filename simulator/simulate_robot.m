function simulate_robot()
    %% Define Fixed Parameters
    %left leg is blue and right leg is red
    % Masses and Inertias for left leg, right leg, and body
    m1 = 0.2393; m2 = 0.0368; m3 = 0.00783; m4 = 0.0155; 
    m5 = 0.2393; m6 = 0.0368; m7 = 0.00783; m8 = 0.0155; % Right leg (same as left leg)
    m_body = 0.1;
    I1 = 25.1e-6; I2 = 53.5e-6; I3 = 9.25e-6; I4 = 22.176e-6; 
    I5 = 25.1e-6; I6 = 53.5e-6; I7 = 9.25e-6; I8 = 22.176e-6; % Right leg
    I_body = 25.1e-6;
    
    % Link lengths and distances from joints to centers of mass
    l_OA = 0.011; l_OB = 0.042; l_AC = 0.096; l_DE = 0.091; l_body = 0.5;
    l_O_m1 = 0.032; l_B_m2 = 0.0344; l_A_m3 = 0.0622; l_C_m4 = 0.0610;
    l_B_m_body = l_body / 2;

    % Motor and gravity parameters
    N = 18.75; Ir = 0.0035 / N^2; g = 9.8;

    % Updated parameter vector to match all defined parameters in derive_robot.m
    p = [m1 m2 m3 m4 m5 m6 m7 m8 m_body I1 I2 I3 I4 I5 I6 I7 I8 I_body Ir N ...
         l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body l_OA l_OB l_AC l_DE l_body g]';

    %% Initial Conditions
    x_base = 0; y_base = 0.3; th1 = -pi/4; th2 = pi/2; th3 = -pi/4; th4 = pi/2; th5 = 0;
    dx_base = 0; dy_base = 0; dth1 = 0; dth2 = 0; dth3 = 0; dth4 = 0; dth5 = 0;
    z0 = [x_base; y_base; th1; th2; th3; th4; th5; dx_base; dy_base; dth1; dth2; dth3; dth4; dth5];

    %% Simulation Parameters
    dt = 0.0005; tf = 1; tspan = 0:dt:tf;
    num_steps = length(tspan);
    z_out = zeros(14, num_steps);
    z_out(:,1) = z0;

    %% Simulation Loop
    for i = 1:num_steps-1
        t = tspan(i);
        z = z_out(:,i);
        dz = dynamics(t, z, p);
        z_out(:,i+1) = z + dz * dt;
    end
%% Compute and Plot Energy
% Compute energy for each time step
E = zeros(1, length(tspan));
for i = 1:length(tspan)
    z = z_out(:, i);  % Extract state at each time step
    E(i) = energy_leg(z, p);  % Compute energy at this state
end

% Plot the total energy over time
figure(2); clf;
plot(tspan, E, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Total Energy (J)');
title('Total Energy of the Robot Over Time');
grid on;

    %% Animate the Robot
    animate_robot(tspan, z_out, p);
end

function dz = dynamics(t, z, p)
    q = z(1:7);
    dq = z(8:14);
    tau = zeros(4,1);  % Control torques are zero (unactuated)

    % Define external forces on left and right feet
    Fx1 = 0; Fy1 = 0;  % Example values for forces on left foot
    Fx2 = 0; Fy2 = 0;  % Example values for forces on right foot
    F_left = [Fx1; Fy1];
    F_right = [Fx2; Fy2];

    % Compute mass matrix and dynamics vector
    A = A_leg(z, p);
    b = b_leg(z, tau, F_left, F_right, p);  % Pass external forces

    % Compute contact forces
    QFc = compute_contact_forces(z, p, Fx1, Fy1, Fx2, Fy2);

    % Accelerations
    ddq = A \ (b + QFc);

    % Form dz
    dz = [dq; ddq];
end

function QFc = compute_contact_forces(z, p, Fx1, Fy1, Fx2, Fy2)
    QFc = zeros(7,1);
    y_base = z(2);
    dy_base = z(9);
    rE_left = position_left_foot(z, p);
    rE_right = position_right_foot(z, p);
    drE_left = velocity_left_foot(z, p);
    drE_right = velocity_right_foot(z, p);

    ground_level = 0;
    Kc = 100; Dc = 50;

    % Base contact
    if y_base <= ground_level
        delta = ground_level - y_base;
        delta_dot = -dy_base;
        Fc_base = max(Kc * delta + Dc * delta_dot, 0);
        QFc_base = [0; Fc_base; zeros(5,1)];
    else
        QFc_base = zeros(7,1);
    end

    % Left foot contact
    J_left = jacobian_left_foot(z, p);  % Retrieve left foot Jacobian
    if rE_left(2) <= ground_level
        delta = ground_level - rE_left(2);
        delta_dot = -drE_left(2);
        Fc_left = max(Kc * delta + Dc * delta_dot, 0);
        QFc_left = J_left' * [0; Fc_left] + J_left' * [Fx1; Fy1];
    else
        QFc_left = J_left' * [Fx1; Fy1];
    end

    % Right foot contact
    J_right = jacobian_right_foot(z, p);  % Retrieve right foot Jacobian
    if rE_right(2) <= ground_level
        delta = ground_level - rE_right(2);
        delta_dot = -drE_right(2);
        Fc_right = max(Kc * delta + Dc * delta_dot, 0);
        QFc_right = J_right' * [0; Fc_right] + J_right' * [Fx2; Fy2];
    else
        QFc_right = J_right' * [Fx2; Fy2];
    end

    QFc = QFc_base + QFc_left + QFc_right;
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
        delete(h_left_leg2);
        delete(h_right_leg2);

        pause(0.01);
    end
end

