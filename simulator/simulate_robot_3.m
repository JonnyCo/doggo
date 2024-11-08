function dz = dynamics(t, z, p, tau)
    q = z(1:7);
    dq = z(8:14);

    % Extract control torques for each leg
    tau_left = tau(1:2);
    tau_right = tau(3:4);

    % Define external forces on left and right feet (horizontal and vertical components)
    [Fc_left, Fc_right] = contact_forces(z, p); % zero

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
function qdot_ground = discrete_impact_contact(z, p, rest_coeff, fric_coeff, ground_height)
    % get positions and velocities
    rE_left  = position_left_foot(z, p);
    rE_right = position_right_foot(z, p);
    vE_left  = velocity_left_foot(z, p);
    vE_right = velocity_right_foot(z, p);
    
    % Extract current velocities
    qdot = z(8:14); % Full velocity vector
    
    % Mass matrix and inverse
    A = A_leg(z, p);
    invA = inv(A);
    
    % Foot height relative to the ground
    Cy_left  = rE_left(2) - ground_height;
    Cy_right = rE_right(2) - ground_height;
    
    % Vertical velocities
    dCy_left  = vE_left(2);
    dCy_right = vE_right(2);
    
    % Initialize contact impulse changes
    dq_impact = zeros(size(qdot));
    
    % Left foot impact handling
    if (Cy_left < 0 && dCy_left < 0)
        J_left = jacobian_left_foot(z, p);
        
        % Vertical force calculation
        Jy_left = J_left(2,:);
        Ly_left = 1 / (Jy_left * invA * Jy_left');
        Fcy_left = Ly_left * (-rest_coeff * dCy_left - Jy_left * qdot);
        
        % Horizontal force with friction limit
        Jx_left = J_left(1,:);
        Lx_left = 1 / (Jx_left * invA * Jx_left');
        Fcx_left = Lx_left * (-Jx_left * qdot);
        
        % Apply friction cone
        if abs(Fcx_left) > fric_coeff * Fcy_left
            Fcx_left = fric_coeff * Fcy_left * sign(Fcx_left);
        end
        
        % Total impulse change from left foot impact
        dq_impact = dq_impact + invA * (Jy_left' * Fcy_left + Jx_left' * Fcx_left);
    end
    
    % Right foot impact handling
    if (Cy_right < 0 && dCy_right < 0)
        J_right = jacobian_right_foot(z, p);
        
        % Vertical force calculation
        Jy_right = J_right(2,:);
        Ly_right = 1 / (Jy_right * invA * Jy_right');
        Fcy_right = Ly_right * (-rest_coeff * dCy_right - Jy_right * qdot);
        
        % Horizontal force with friction limit
        Jx_right = J_right(1,:);
        Lx_right = 1 / (Jx_right * invA * Jx_right');
        Fcx_right = Lx_right * (-Jx_right * qdot);
        
        % Apply friction cone
        if abs(Fcx_right) > fric_coeff * Fcy_right
            Fcx_right = fric_coeff * Fcy_right * sign(Fcx_right);
        end
        
        % Total impulse change from right foot impact
        dq_impact = dq_impact + invA * (Jy_right' * Fcy_right + Jx_right' * Fcx_right);
    end
    
    % Update the velocity in the state vector with the impact changes
    qdot_ground = qdot + dq_impact;
end
function qdot_joint = joint_limit_constraint(z, p, rest_coeff2, fric_coeff2, jointConstraint1, jointConstraint2, bodyConstraint)
    % Extract joint velocities from state vector
    qdot_joint = z(8:14);
    
    % Joint angles
    th1 = z(3);   % Left hip joint
    th2 = z(4);   % Left knee joint
    th3 = z(5);   % Right hip joint
    th4 = z(6);   % Right knee joint
    th5 = z(7);   % Body angle
    
    % Joint velocities
    dth1 = z(10); % Left hip joint velocity
    dth2 = z(11); % Left knee joint velocity
    dth3 = z(12); % Right hip joint velocity
    dth4 = z(13); % Right knee joint velocity
    dth5 = z(14); % Body angle velocity
    
    % Joint angles and velocities as vectors (include th5 and dth5)
    jointAngles = [th1; th2; th3; th4; th5];
    jointVelocities = [dth1; dth2; dth3; dth4; dth5];
    
    % Mass matrix and its inverse
    A = A_leg(z, p);
    invA = inv(A);
    
    % Indices of joints in qdot_joint (adjusted for qdot_joint indexing)
    % qdot_joint indices: [dx, dy, dth1, dth2, dth3, dth4, dth5]
    joint_indices = [3, 4, 5, 6, 7]; % Indices corresponding to dth1 to dth5
    
    % Loop over each joint to check constraint violation
    for idx = 1:length(jointAngles)
        angle = jointAngles(idx);
        velocity = jointVelocities(idx);
        joint_idx = joint_indices(idx);
        
        % Lower constraint violation
        if joint_idx ~= 7
            if joint_idx == 4
                jointConstraintTrue1 = 0.1;
                jointConstraintTrue2 = jointConstraint2;
            elseif joint_idx == 6
                jointConstraintTrue1 = 0.1;
                jointConstraintTrue2 = jointConstraint2;
            else
                jointConstraintTrue1 = jointConstraint1;
                jointConstraintTrue2 = jointConstraint2;
            end


            if (angle < jointConstraintTrue1 && velocity < 0)
                % Define Jacobian for the current joint
                J = zeros(1, length(qdot_joint));
                J(joint_idx) = 1;
                
                % Compute scalar multiplier
                L = 1 / (J * invA * J');
                
                % Corrective force
                dC = angle - jointConstraintTrue1;
                Fc = L * (-rest_coeff2 * dC - J * qdot_joint);
                
                % Update joint velocities
                qdot_joint = qdot_joint + invA * J' * Fc;
            end
            
            % Upper constraint violation
            if (angle > jointConstraintTrue2 && velocity > 0)
                % Define Jacobian for the current joint
                J = zeros(1, length(qdot_joint));
                J(joint_idx) = 1;
                
                % Compute scalar multiplier
                L = 1 / (J * invA * J');
                
                % Corrective force
                dC = angle - jointConstraintTrue2;
                Fc = L * (-rest_coeff2 * dC - J * qdot_joint);
                
                % Update joint velocities
                qdot_joint = qdot_joint + invA * J' * Fc;
            end
        else
            if (angle < -bodyConstraint && velocity < 0)
                % Define Jacobian for the current joint
                J = zeros(1, length(qdot_joint));
                J(joint_idx) = 1;
                
                % Compute scalar multiplier
                L = 1 / (J * invA * J');
                
                % Corrective force
                Fc = L * (-rest_coeff2 * (J * qdot_joint)' - fric_coeff2 * sign(J * qdot_joint)');
                
                % Update joint velocities
                qdot_joint = qdot_joint + invA * J' * Fc;
            end
            
            % Upper constraint violation
            if (angle > bodyConstraint && velocity > 0)
                % Define Jacobian for the current joint
                J = zeros(1, length(qdot_joint));
                J(joint_idx) = 1;
                
                % Compute scalar multiplier
                L = 1 / (J * invA * J');
                
                % Corrective force
                Fc = L * (-rest_coeff2 * (J * qdot_joint)' - fric_coeff2 * sign(J * qdot_joint)');
                
                % Update joint velocities
                qdot_joint = qdot_joint + invA * J' * Fc;
            end
        end
    end
end
function [Fc_left, Fc_right] = contact_forces(z, p)
    Fc_right = [0; 0];
    Fc_left = [0; 0];
end
function tau = control_law(t, z, p, dt, genome)
    % Define torsional spring and damper parameters
    K_torsional_L = genome(7);    % Torsional spring constant
    K_torsional_R = genome(8);    % Torsional spring constant
    D_torsional_L = genome(9);  % Damping constant
    D_torsional_R = genome(10);  % Damping constant
    max_torque = 3;     % Max torque limit

    % K_torsional_L = 4;
    % K_torsional_R = 4;
    % D_torsional_L = 0.2;
    % D_torsional_R = 0.2;

    % Extract joint angles and velocities
    th1 = z(3);   % Left hip joint
    th2 = z(4);   % Left knee joint
    th3 = z(5);   % Right hip joint
    th4 = z(6);   % Right knee joint
    dth1 = z(10); % Left hip joint velocity
    dth2 = z(11); % Left knee joint velocity
    dth3 = z(12); % Right hip joint velocity
    dth4 = z(13); % Right knee joint velocity

    % Get desired joint angles
    [th1_des, th2_des, th3_des, th4_des] = get_desired_joint_angles(t, genome);

    % Virtual torsional spring-damper torques for each joint
    tau1 = -K_torsional_L * (th1 - th1_des) - D_torsional_L * dth1;  % Left hip
    tau2 = -K_torsional_L * (th2 - th2_des) - D_torsional_L * dth2;  % Left knee
    tau3 = -K_torsional_R * (th3 - th3_des) - D_torsional_R * dth3;  % Right hip
    tau4 = -K_torsional_R * (th4 - th4_des) - D_torsional_R * dth4;  % Right knee

    % Clamp torques to prevent excessive values
    tau1 = max(-max_torque, min(tau1, max_torque));
    tau2 = max(-max_torque, min(tau2, max_torque));
    tau3 = max(-max_torque, min(tau3, max_torque));
    tau4 = max(-max_torque, min(tau4, max_torque));

    % Combine torques into a single vector
    tau = [tau1; tau2; tau3; tau4];

end
function [th1_des, th2_des, th3_des, th4_des] = get_desired_joint_angles(t, genome)
    % Optimization parameters
    omega = genome(1); % angular frequency (for all)
    hip_amplitude_left = genome(2);
    hip_amplitude_right = genome(2);
    knee_amplitude_left = genome(4);
    knee_amplitude_right = genome(4);
    phaseOffset = genome(6);

    shift1 = genome(11);
    shift2 = genome(12);
    shift3 = genome(11);
    shift4 = genome(12);

    % Interpolate desired angles
    th1_des = shift1 + hip_amplitude_left * cos(omega * t);             % Left hip follows a cosine wave around initial angle
    th2_des = shift2 + knee_amplitude_left * sin(omega * t);             % Left knee follows a sine wave around initial angle
    th3_des = shift3 + hip_amplitude_right * cos(omega * t + phaseOffset); % Right hip, 180 degrees out of phase
    th4_des = shift4 + knee_amplitude_right * sin(omega * t + phaseOffset); % Right knee, 180 degrees out of phase

end
function animate_robot(tspan, z_out, p, genome)
    % Set up video writer with frame rate based on output window updates
    fps = 1 / 0.01;  % Assuming a pause of 0.01 seconds per frame
    video = VideoWriter('robot_animation.mp4', 'MPEG-4');
    video.FrameRate = fps;

    % Double the resolution by setting custom resolution for the video
    video.Quality = 100;  % Set to maximum quality
    open(video);

    % Set up the figure with double resolution
    figure(1); clf; hold on;
    set(gcf, 'Position', [100, 100, 1600, 900]);  % Double the resolution (e.g., 1920x1080 for Full HD)

    % Set larger font sizes for all text in the figure
    fontSize = 16;  % Define a larger font size

    % Labels and title with increased font size
    xlabel('X Position (m)', 'FontSize', fontSize); 
    ylabel('Y Position (m)', 'FontSize', fontSize);
    title('Robot Animation', 'FontSize', fontSize + 2);  % Slightly larger for the title

    % Set tick direction, location, and make ticks only on the left and bottom
    set(gca, 'FontSize', fontSize, 'TickDir', 'in', 'Box', 'on', ...
         'XAxisLocation', 'bottom', 'YAxisLocation', 'left');

    axis equal;  % Ensures 1:1 scaling on both axes
    yticks(-0.1:0.1:0.5);  % Set y-axis ticks as multiples of 0.1

    % Plot the ground line
    plot([-50, 50], [0, 0], 'LineWidth', 2, 'Color', [0, 0, 0]); % Ground line
    plot([-50, 50], [-0.0505, -0.0505], 'LineWidth', 92, 'Color', [0.2588, 0.2510, 0.2431]); % Ground line

    % Add randomly scattered dots on the floor for visual reference
    num_dots = 500;  % Number of dots to scatter
    x_dots = -50 + 100 * rand(1, num_dots);  % Random x positions within -50 to 50 range
    y_dots = -0.05 * rand(1, num_dots);    % Small variation in y to simulate being on the ground
    scatter(x_dots, y_dots, 50, 'filled', 'MarkerFaceColor', [0, 0, 0]);

    % Initialize plot handles for the body, left leg, and right leg
    h_body1 = plot([0, 0], [0, 0], 'LineWidth', 40, 'Color', [0.4314, 0.3176, 0.1804]);
    h_body2 = plot([0, 0], [0, 0], 'LineWidth', 6, 'Color', [0.8275, 0.5843, 0.2784]);
    
    % Initialize plot handles for left and right leg segments with specified colors
    h_OB_left = plot([0],[0],'LineWidth',4, 'Color', [0.5529, 0.7647, 0.7804]);
    h_AC_left = plot([0],[0],'LineWidth',4, 'Color', [0.5529, 0.7647, 0.7804]);
    h_BD_left = plot([0],[0],'LineWidth',4, 'Color', [0.5529, 0.7647, 0.7804]);
    h_CE_left = plot([0],[0],'LineWidth',4, 'Color', [0.5529, 0.7647, 0.7804]);

    h_OB_right = plot([0],[0],'LineWidth',4, 'Color', [0.3255, 0.4392, 0.4431]);
    h_AC_right = plot([0],[0],'LineWidth',4, 'Color', [0.3255, 0.4392, 0.4431]);
    h_BD_right = plot([0],[0],'LineWidth',4, 'Color', [0.3255, 0.4392, 0.4431]);
    h_CE_right = plot([0],[0],'LineWidth',4, 'Color', [0.3255, 0.4392, 0.4431]);

    % Initialize plot handles for tracking the paths of points E_left and E_right
    h_trace_left = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', [0.5529, 0.7647, 0.7804], 'LineStyle', ':');
    h_trace_right = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', [0.3255, 0.4392, 0.4431], 'LineStyle', ':');
    
    % Initialize plot handles for desired foot positions and traces
    h_desired_left = plot(NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.5529, 0.7647, 0.7804], 'MarkerEdgeColor', [0.5529, 0.7647, 0.7804]);
    h_desired_right = plot(NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.3255, 0.4392, 0.4431], 'MarkerEdgeColor', [0.3255, 0.4392, 0.4431]);
    %h_trace_desired_left = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'y', 'LineStyle', '--');
    %h_trace_desired_right = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'y', 'LineStyle', '--');
    
    % Define fixed box width and height around the robot
    box_width = 1; % Width of the box around the robot
    box_height = 0.5; % Height of the box around the robot

    % Add a text object to display the time in the top right corner with increased font size
    time_display = text(0.85, 0.9, '0.00 s', 'Units', 'normalized', ...
                        'HorizontalAlignment', 'left', 'FontSize', fontSize, 'FontWeight', 'bold');

    dist_display = text(0.85, 0.85, '0.00 m', 'Units', 'normalized', ...
                        'HorizontalAlignment', 'left', 'FontSize', fontSize, 'FontWeight', 'bold');

    speed_display = text(0.85, 0.8, '0.00 m/s', 'Units', 'normalized', ...
                        'HorizontalAlignment', 'left', 'FontSize', fontSize, 'FontWeight', 'bold');

    for i = 1:10:length(tspan)
        z = z_out(:, i);
        startTime = 0;
        current_time = tspan(i);  % Current simulation time in seconds

        if current_time == startTime
            keypoints = keypoints_leg(z, p);
            rO_left = keypoints(:, 11);       % Left hip (O)
            rOR = keypoints(:, 12);           % Right hip (OR)

            origionalDistance = (rO_left(1) + rOR(1))/2;
        end

        if current_time >= startTime

            clockTime = current_time-startTime;

            % Update the time display
            set(time_display, 'String', sprintf('%.2f s', clockTime));
    
            % Get keypoints for visualization
            keypoints = keypoints_leg(z, p);
    
            % Extract key points for the left leg, right leg, and body
            % Left leg keypoints
            rO_left = keypoints(:, 11);       % Left hip (O)
            rA_left = keypoints(:, 1);        % Left thigh joint (A)
            rB_left = keypoints(:, 2);        % Intermediate point (B) on left leg
            rC_left = keypoints(:, 3);        % Left knee joint (C)
            rD_left = keypoints(:, 4);        % Intermediate point (D) on left leg
            rE_left = keypoints(:, 5);        % Left foot (E)
    
            % Right leg keypoints
            rOR = keypoints(:, 12);           % Right hip (OR)
            rA_right = keypoints(:, 6);       % Right thigh joint (AR)
            rB_right = keypoints(:, 7);       % Intermediate point (B) on right leg
            rC_right = keypoints(:, 8);       % Right knee joint (CR)
            rD_right = keypoints(:, 9);       % Intermediate point (D) on right leg
            rE_right = keypoints(:, 10);      % Right foot (ER)
    
            % Update the trace for point E_left and E_right
            set(h_trace_left, 'XData', [get(h_trace_left, 'XData'), rE_left(1)], ...
                              'YData', [get(h_trace_left, 'YData'), rE_left(2)]);
            set(h_trace_right, 'XData', [get(h_trace_right, 'XData'), rE_right(1)], ...
                               'YData', [get(h_trace_right, 'YData'), rE_right(2)]);
    
            % Compute desired joint angles
            [th1_des, th2_des, th3_des, th4_des] = get_desired_joint_angles(current_time, genome);
    
            % Construct desired state vector
            z_desired = z;
            z_desired(3) = th1_des;
            z_desired(4) = th2_des;
            z_desired(5) = th3_des;
            z_desired(6) = th4_des;
            z_desired(10:13) = 0;  % Zero velocities for desired state
    
            % Get keypoints for desired positions
            keypoints_desired = keypoints_leg(z_desired, p);
            rE_left_desired = keypoints_desired(:, 5);   % Desired left foot position
            rE_right_desired = keypoints_desired(:, 10); % Desired right foot position
    
            % Update the desired foot position plots
            set(h_desired_left, 'XData', rE_left_desired(1), 'YData', rE_left_desired(2));
            set(h_desired_right, 'XData', rE_right_desired(1), 'YData', rE_right_desired(2));
    
    
            % Define body linkage (from left hip to right hip)
            % Calculate the current length of the line
            dx = rOR(1) - rO_left(1);
            dy = rOR(2) - rO_left(2);
            line_length = sqrt(dx^2 + dy^2);
            
            % Calculate the extension amount (5% of the original length)
            extension = 0.05 * line_length;
            
            % Calculate the unit direction vector for the line
            direction_x = dx / line_length;
            direction_y = dy / line_length;
            
            % Extend both endpoints
            new_rO_left_x = rO_left(1) - extension * direction_x;
            new_rO_left_y = rO_left(2) - extension * direction_y;
            new_rOR_x = rOR(1) + extension * direction_x;
            new_rOR_y = rOR(2) + extension * direction_y;
            
            % Update the plot with the extended line
            set(h_body1, 'XData', [new_rO_left_x, new_rOR_x], 'YData', [new_rO_left_y, new_rOR_y]);
            set(h_body2, 'XData', [rO_left(1), rOR(1)], 'YData', [rO_left(2), rOR(2)]);

            % Determine distance
            currentDistance = (rO_left(1) + rOR(1))/2 - origionalDistance;
            set(dist_display, 'String', sprintf('%.2f m', abs(currentDistance)));

            aveSpeed = currentDistance / clockTime;
            set(speed_display, 'String', sprintf('%.2f m/s', abs(aveSpeed)));
            
            % Update left leg segment plots
            set(h_OB_left, 'XData', [rO_left(1), rB_left(1)], 'YData', [rO_left(2), rB_left(2)]);
            set(h_AC_left, 'XData', [rA_left(1), rC_left(1)], 'YData', [rA_left(2), rC_left(2)]);
            set(h_BD_left, 'XData', [rB_left(1), rD_left(1)], 'YData', [rB_left(2), rD_left(2)]);
            set(h_CE_left, 'XData', [rC_left(1), rE_left(1)], 'YData', [rC_left(2), rE_left(2)]);
    
            % Update right leg segment plots
            set(h_OB_right, 'XData', [rOR(1), rB_right(1)], 'YData', [rOR(2), rB_right(2)]);
            set(h_AC_right, 'XData', [rA_right(1), rC_right(1)], 'YData', [rA_right(2), rC_right(2)]);
            set(h_BD_right, 'XData', [rB_right(1), rD_right(1)], 'YData', [rB_right(2), rD_right(2)]);
            set(h_CE_right, 'XData', [rC_right(1), rE_right(1)], 'YData', [rC_right(2), rE_right(2)]);
    
            % Update the axis to keep a fixed box around the robot's horizontal position
            x_center = z(1); % Robot's horizontal position (x_base)
            axis([x_center - box_width/2, x_center + box_width/2, -0.1, box_height - 0.1]);
            
            % Capture the current frame
            frame = getframe(gcf);
            writeVideo(video, frame);  % Write the frame to the video
    
            % Pause for a moment to match frame rate
            pause(0.01);
        end
    end

    % Close the video writer
    close(video);
end
function optimize_robot()
    %% Define Lower and Upper Bounds for Genome Parameters
    lb = zeros(10,1);
    lb(1) = 1;            % omega
    lb(2) = 0;            % hip_amplitude_left
    lb(3) = 0;            % hip_amplitude_right
    lb(4) = 0;            % knee_amplitude_left
    lb(5) = 0;            % knee_amplitude_right
    lb(6) = 0;            % phase offset
    lb(7:8) = 0;          % K_torsional_L and K_torsional_R
    lb(9:10) = 0;         % D_torsional_L and D_torsional_R

    lb(11:14) = -0.34;

    ub = zeros(10,1);
    ub(1) = 20;           
    ub(2) = 0.75;         
    ub(3) = 0.75;         
    ub(4) = 0.75;         
    ub(5) = 0.75;         
    ub(6) = 2*pi;           
    ub(7:8) = 20;          % K_torsional_L and K_torsional_R
    ub(9:10) = 1;         % D_torsional_L and D_torsional_R

    ub(11:14) = 0.34;

    %% Initial Guess
    x0 = [15, pi/8, pi/8, pi/12, pi/12, pi, 4, 4, 0.2, 0.2];  % Start from the middle of the bounds

    %% Optimization Options
    %options = optimoptions('simulannealbnd', 'Display', 'iter', 'MaxIterations', 1000);
    options = optimoptions('ga', 'Display', 'iter', 'MaxGenerations', 30, 'PopulationSize', 50);

    %% Run Optimization
    %[genome_opt, fval] = simulannealbnd(@objective_function, x0, lb, ub, options);
    [genome_opt, fval] = ga(@objective_function, 14, [], [], [], [], lb, ub, [], options);

    %% Best Distance Achieved
    best_distance = -fval;

    %% Save the Best Genome
    save('best_genome.mat', 'genome_opt', 'best_distance');

    %% Run Simulation with Optimal Genome and Animation
    simulate_robot(genome_opt, true, 0.001, 20);
end
function obj = objective_function(genome)
    try
        %% Run Simulation without Animation for Speed
        [distance, max_th5_deviation] = simulate_robot(genome, false, 0.001, 20);
        if isnan(distance) || isinf(distance)
            obj = 1e6;  % Penalty for invalid distance
            return;
        end
        
        % Penalty parameters
        th5_limit_deg = 10;  % Limit in degrees
        penalty_weight = 1e3; % Weight of the penalty in the objective function
        
        % Check if max deviation exceeds the limit
        if max_th5_deviation > th5_limit_deg
            % Calculate penalty
            penalty = penalty_weight * (max_th5_deviation - th5_limit_deg)^2;
        else
            penalty = 0;
        end
        
        % Compute objective (negative distance plus penalty)
        obj = -abs(distance) + penalty;
        
    catch ME
        disp('An error occurred during simulation:');
        disp(ME.message);
        obj = 1e6;  % Penalty for simulation failure
    end
end
function [distance, max_th5_deviation] = simulate_robot(genome, do_animate, dt, tf)
    %% Define Fixed Parameters
    %left leg is blue and right leg is red
    % Masses and Inertias for left leg, right leg, and body
    m1 = 0.2393; m2 = 0.0368; m3 = 0.00783; m4 = 0.0155; 
    m5 = 0.2393; m6 = 0.0368; m7 = 0.00783; m8 = 0.0155; % Right leg (same as left leg)
    m_body = 0.2;
    I1 = 25.1e-6; I2 = 53.5e-6; I3 = 9.25e-6; I4 = 22.176e-6; 
    I5 = 25.1e-6; I6 = 53.5e-6; I7 = 9.25e-6; I8 = 22.176e-6; % Right leg
    I_body = 100e-6;
    
    % Link lengths and distances from joints to centers of mass
    l_OA = 0.011; l_OB = 0.042; l_AC = 0.096; l_DE = 0.091; l_body = 0.25;
    l_O_m1 = 0.032; l_B_m2 = 0.0344; l_A_m3 = 0.0622; l_C_m4 = 0.0610;
    l_B_m_body = l_body / 2;

    % Motor and gravity parameters
    N = 18.75; Ir = 0.0035 / N^2; g = 10;% 9.8;

    % floor
    rest_coeff = 0.0;
    fric_coeff = 1;
    ground_height = 0;

    % joint
    rest_coeff2 = 0.;
    jointConstraint1 = deg2rad(-40);
    fric_coeff2 = 1;
    jointConstraint2 = deg2rad(40);

    bodyConstraint = pi/4;

    % Updated parameter vector to match all defined parameters in derive_robot.m
    p = [m1 m2 m3 m4 m5 m6 m7 m8 m_body I1 I2 I3 I4 I5 I6 I7 I8 I_body Ir N ...
         l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_B_m_body l_OA l_OB l_AC l_DE l_body g]';

    %% Initial Conditions
    x_base = 0; y_base = .25; th1 = -pi/4; th2 = pi/2; th3 = -pi/4; th4 = pi/2; th5 = 0;
    dx_base = -.02; dy_base = 0; dth1 = 0; dth2 = 0; dth3 = 0; dth4 = 0; dth5 = 0;

    %% Simulation Parameters   
    tspan = 0:dt:tf;
    num_steps = length(tspan);
    z_out = zeros(14, num_steps);
    z_out(:,1) = [x_base; y_base; th1; th2; th3; th4; th5; dx_base; dy_base; dth1; dth2; dth3; dth4; dth5];

    %% Simulation Loop
    for i = 1:num_steps-1
        t = tspan(i);

        % Control
        tau = control_law(t, z_out(:,i), p, dt, genome);

        % Dynamics
        dz = dynamics(t, z_out(:, i), p, tau);

        % Velocity Update
        z_next = z_out(:,i) + dz * dt;
        
        z_next(8:14) = discrete_impact_contact(z_next, p, rest_coeff, fric_coeff, ground_height);

        z_next(8:14) = joint_limit_constraint(z_next, p, rest_coeff2, fric_coeff2, jointConstraint1, jointConstraint2, bodyConstraint);

        

        % State Update
        z_out(:, i+1) = z_next;

        % Check for Simulation Failure
        % if any(isnan(z_out(:,i+1))) || any(isinf(z_out(:,i+1))) || z_out(2,i+1) < -0.1
        %     distance = -Inf;  % Indicate failure
        %     max_th5_deviation = Inf;
        %     return;
        % end

        % Additional failure checks during t > 10s
        if t > 10
            keypoints = keypoints_leg(z_out(:, i+1), p);
            % Extract key points for the left leg, right leg, and body
            % Left leg keypoints
            rO_left = keypoints(:, 11);       % Left hip (O)
            rA_left = keypoints(:, 1);        % Left thigh joint (A)
            rB_left = keypoints(:, 2);        % Intermediate point (B) on left leg
            rC_left = keypoints(:, 3);        % Left knee joint (C)
            rD_left = keypoints(:, 4);        % Intermediate point (D) on left leg
            rE_left = keypoints(:, 5);        % Left foot (E)
    
            % Right leg keypoints
            rOR = keypoints(:, 12);           % Right hip (OR)
            rA_right = keypoints(:, 6);       % Right thigh joint (AR)
            rB_right = keypoints(:, 7);       % Intermediate point (B) on right leg
            rC_right = keypoints(:, 8);       % Right knee joint (CR)
            rD_right = keypoints(:, 9);       % Intermediate point (D) on right leg
            rE_right = keypoints(:, 10);      % Right foot (ER)
    

            if rO_left(2) < 0 || rA_left(2) < 0 || rB_left(2) < 0 || rC_left(2) < 0 || rD_left(2) < 0 || ...
                rOR(2) < 0 || rA_right(2) < 0 || rB_right(2) < 0 || rC_right(2) < 0 || rD_right(2) < 0
                distance = -Inf; % failure from going below table
                max_th5_deviation = Inf;
                return;
            end
        end
    end

    %% Compute Distance Traveled Between t=10s and t=20s
    idx_t10 = round(10 / dt) + 1;
    idx_t20 = round(20 / dt) + 1;
    x_base_t10 = z_out(1, idx_t10);
    x_base_t20 = z_out(1, idx_t20);
    distance = x_base_t20 - x_base_t10;

    %% Compute Maximum Absolute Deviation of th5 Between 10s and 20s
    th5_values = z_out(7, idx_t10:idx_t20); % Extract th5 during 10s to 20s
    th5_values_deg = rad2deg(th5_values);   % Convert to degrees
    max_th5_deviation = max(abs(th5_values_deg)); % Maximum absolute deviation

    %% Optional Animation
    if do_animate
        animate_robot(tspan, z_out, p, genome);
    end
end


%x0 = [-15, pi/8, pi/8, pi/12, pi/12, pi, 4, 4, 0.2, 0.2];  % Start from the middle of the bounds
genome = [9.484985070090184	0.3649633885327803	0.6215508826869114	0.375	0.7308591786108478	1.171454665034373	7.660431538575179	9.922501627831469	0.5948633904087623	1.5715346233403638	0.30899037422521464	-0.5776245780214992	-0.2920169615927195	-0.9739408329871375];
dt = 0.001;
tf = 20;
%simulate_robot(genome, true, dt, tf)
optimize_robot();
