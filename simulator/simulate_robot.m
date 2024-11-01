function simulate_robot()
    %% Define Fixed Parameters (using provided values)
    m1 = 0.2393;          % Mass of link 1 (kg)
    m2 = 0.0368;          % Mass of link 2 (kg)
    m3 = 0.00783;         % Mass of link 3 (kg)
    m4 = 0.0155;          % Mass of link 4 (kg)
    m_body = .1;          % Mass of the body (kg)

    I1 = 25.1e-6;         % Inertia of link 1 (kg*m^2)
    I2 = 53.5e-6;         % Inertia of link 2 (kg*m^2)
    I3 = 9.25e-6;         % Inertia of link 3 (kg*m^2)
    I4 = 22.176e-6;       % Inertia of link 4 (kg*m^2)
    I_body = 25.1e-6;           % Inertia of the body (kg*m^2)

    l_OA = 0.011;         % Length from joint O to point A (m)
    l_OB = 0.042;         % Length from joint O to point B (m)
    l_AC = 0.096;         % Length from point A to point C (m)
    l_DE = 0.091;         % Length from point D to point E (m)
    l_body = 0.5;         % Length of the body (m)

    l_O_m1 = 0.032;       % Distance from O to CoM of link 1 (m)
    l_B_m2 = 0.0344;      % Distance from B to CoM of link 2 (m)
    l_A_m3 = 0.0622;      % Distance from A to CoM of link 3 (m)
    l_C_m4 = 0.0610;      % Distance from C to CoM of link 4 (m)
    l_B_m_body = l_body / 2; % Distance from B to CoM of the body (m)

    N = 18.75;            % Gear ratio
    Ir = 0.0035 / N^2;    % Rotor inertia

    g = 9.81;             % Gravitational acceleration (m/s^2)

    %% Parameter vector as per updated derive_leg.m
    p = [m1 m2 m3 m4 m_body I1 I2 I3 I4 I_body Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body ...
         l_OA l_OB l_AC l_DE l_body g]';

    %% Initial conditions
    % Generalized coordinates [x_base; y_base; th1; th2; th3; th4; th5]
    x_base = 0;       % Initial horizontal position of the base
    y_base = 1;       % Initial vertical position of the base (1 meter)
    th1 = pi/4;      % Left leg hip angle
    th2 = -pi/2;       % Left leg knee angle
    th3 = pi/4;      % Right leg hip angle
    th4 = -pi/2;       % Right leg knee angle
    th5 = 0;          % Body angle

    % Initial velocities
    dx_base = 0;
    dy_base = 0;
    dth1 = 0;
    dth2 = 0;
    dth3 = 0;
    dth4 = 0;
    dth5 = 0;

    % Initial state vector
    z0 = [x_base; y_base; th1; th2; th3; th4; th5; dx_base; dy_base; dth1; dth2; dth3; dth4; dth5];

    %% Simulation parameters
    dt = 0.001;            % Time step (s)
    tf = 2;                % Final time (s)
    tspan = 0:dt:tf;
    num_steps = length(tspan);

    % Preallocate state array
    z_out = zeros(14, num_steps);
    z_out(:,1) = z0;

    %% Simulation loop
    for i = 1:num_steps-1
        t = tspan(i);
        z = z_out(:,i);

        dz = dynamics(t, z, p);

        % Euler integration
        z_out(:,i+1) = z + dz * dt;
    end

    %% Animate the robot
    animate_robot(tspan, z_out, p);

end

function dz = dynamics(t, z, p)
    % Extract generalized coordinates and velocities
    q = z(1:7);     % [x_base; y_base; th1; th2; th3; th4; th5]
    dq = z(8:14);   % [dx_base; dy_base; dth1; dth2; dth3; dth4; dth5]

    % Control torques are zero (unactuated)
    tau = zeros(4,1);  % [tau1; tau2; tau3; tau4]

    % Compute mass matrix and dynamics vector
    A = A_leg(z, p);       % Mass matrix (from updated derive_leg.m)
    b = b_leg(z, tau, p);  % Dynamics vector (from updated derive_leg.m)

    % Compute contact forces
    QFc = compute_contact_forces(z, p);

    % Accelerations
    ddq = A \ (b + QFc);

    % Form dz
    dz = [dq; ddq];
end

function QFc = compute_contact_forces(z, p)
    % Initialize contact force vector
    QFc = zeros(7,1);

    % Extract variables
    x_base = z(1);
    y_base = z(2);
    dx_base = z(8);
    dy_base = z(9);

    % Positions and velocities of the left and right feet
    rE_left = position_left_foot(z, p);
    rE_right = position_right_foot(z, p);

    drE_left = velocity_left_foot(z, p);
    drE_right = velocity_right_foot(z, p);

    % Ground level (y = 0)
    ground_level = 0;

    % Contact stiffness and damping
    Kc = 1e5;  % Contact stiffness (N/m)
    Dc = 1e3;  % Contact damping (N/(m/s))

    % Initialize total generalized forces
    QFc_total = zeros(7,1);

    % Base contact (if base can contact the ground)
    if y_base <= ground_level
        % Penetration depth
        delta = ground_level - y_base;
        % Penetration rate
        delta_dot = -dy_base;
        % Normal force
        Fc_base = Kc * delta + Dc * delta_dot;
        % Prevent negative force (no adhesion)
        Fc_base = max(Fc_base, 0);
        % Generalized force due to contact on base
        QFc_base = [0; Fc_base; zeros(5,1)];
    else
        QFc_base = zeros(7,1);
    end

    % Left foot contact
    if rE_left(2) <= ground_level
        % Penetration depth
        delta = ground_level - rE_left(2);
        % Penetration rate
        delta_dot = -drE_left(2);
        % Normal force
        Fc_left = Kc * delta + Dc * delta_dot;
        % Prevent negative force (no adhesion)
        Fc_left = max(Fc_left, 0);
        % Jacobian for left foot
        J_left = jacobian_left_foot(z, p);  % Should be (2 x 7)
        % Generalized forces due to contact
        QFc_left = J_left' * [0; Fc_left];
    else
        QFc_left = zeros(7,1);
    end

    % Right foot contact
    if rE_right(2) <= ground_level
        % Penetration depth
        delta = ground_level - rE_right(2);
        % Penetration rate
        delta_dot = -drE_right(2);
        % Normal force
        Fc_right = Kc * delta + Dc * delta_dot;
        % Prevent negative force (no adhesion)
        Fc_right = max(Fc_right, 0);
        % Jacobian for right foot
        J_right = jacobian_right_foot(z, p);  % Should be (2 x 7)
        % Generalized forces due to contact
        QFc_right = J_right' * [0; Fc_right];
    else
        QFc_right = zeros(7,1);
    end

    % Total contact forces
    QFc = QFc_base + QFc_left + QFc_right;
end
function animate_robot(tspan, z_out, p)
    figure(1); clf;
    hold on
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Robot Dropping Simulation');
    axis equal
    axis([-1 1 -0.5 1.5]);  % Adjust axes as needed

    % Ground line
    plot([-5, 5], [0, 0], 'k--', 'LineWidth', 2);

    % Initialize plot handles
    h_left_leg = plot([0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 'b', 'LineWidth', 2);
    h_right_leg = plot([0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 'r', 'LineWidth', 2);
    h_body = plot([0, 0], [0, 0], 'k', 'LineWidth', 4);

    for i = 1:10:length(tspan)
        z = z_out(:, i);

        % Extract base position
        x_base = z(1);
        y_base = z(2);
        r_base = [x_base; y_base];

        % Get keypoints for visualization
        keypoints = keypoints_leg(z, p);

        % Left leg keypoints
        rA = keypoints(:, 1);       % Left hip
        rB = keypoints(:, 2);       % Left thigh joint
        rC = keypoints(:, 3);       % Left knee
        rD = keypoints(:, 4);       % Left shank joint
        rE_left = keypoints(:, 5);  % Left foot

        % Right leg keypoints
        rA_right = keypoints(:, 6);     % Right hip
        rB_right = keypoints(:, 7);     % Right thigh joint
        rC_right = keypoints(:, 8);     % Right knee
        rD_right = keypoints(:, 9);     % Right shank joint
        rE_right = keypoints(:, 9);     % Right foot (note: keypoints(:,9) is used for both rD_right and rE_right)

        % Body endpoint
        rM = keypoints(:, 10);      % Body endpoint for visualization

        % Left leg points to plot
        left_leg_x = [r_base(1), rA(1), rC(1), rE_left(1)];
        left_leg_y = [r_base(2), rA(2), rC(2), rE_left(2)];

        % Left leg secondary linkage (if applicable)
        left_leg_x2 = [r_base(1), rB(1), rD(1), rE_left(1)];
        left_leg_y2 = [r_base(2), rB(2), rD(2), rE_left(2)];

        % Right leg points to plot
        right_leg_x = [r_base(1), rA_right(1), rC_right(1), rE_right(1)];
        right_leg_y = [r_base(2), rA_right(2), rC_right(2), rE_right(2)];

        % Right leg secondary linkage (if applicable)
        right_leg_x2 = [r_base(1), rB_right(1), rD_right(1), rE_right(1)];
        right_leg_y2 = [r_base(2), rB_right(2), rD_right(2), rE_right(2)];

        % Update left leg plot (primary linkage)
        set(h_left_leg, 'XData', left_leg_x, 'YData', left_leg_y);

        % Update left leg plot (secondary linkage)
        % For visual clarity, we can plot the secondary linkage with a different line style
        h_left_leg2 = plot(left_leg_x2, left_leg_y2, 'b--', 'LineWidth', 1);

        % Update right leg plot (primary linkage)
        set(h_right_leg, 'XData', right_leg_x, 'YData', right_leg_y);

        % Update right leg plot (secondary linkage)
        % For visual clarity, we can plot the secondary linkage with a different line style
        h_right_leg2 = plot(right_leg_x2, right_leg_y2, 'r--', 'LineWidth', 1);

        % Update body plot
        set(h_body, 'XData', [r_base(1), rM(1)], 'YData', [r_base(2), rM(2)]);

        drawnow

        % Delete secondary linkage plots to avoid accumulation
        delete(h_left_leg2);
        delete(h_right_leg2);

        pause(0.01);
    end
end
