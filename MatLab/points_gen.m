%copy from here

% Define parameters
omega = 10;              % Angular frequency for both hip and knee (rad/s)
hip_amp = pi / 8;       % Amplitude of hip oscillation
knee_amp = pi / 12;     % Amplitude of knee oscillation
phase_offset = pi;      % Phase offset for the right leg
num_points = 10;       % Number of points to generate (sets resolution)

% Calculate the period for one full cycle
T = 2 * pi / abs(omega);  % Period for one full cycle

% Define time vector over one period with num_points points
t = linspace(0, T, num_points);  % Time vector for one period

% Compute joint angles
q1_des = -pi/4 + hip_amp * cos(omega * t);             % Left hip
q2_des = pi/2 + knee_amp * sin(omega * t);             % Left knee
q3_des = -pi/4 + hip_amp * cos(omega * t + phase_offset); % Right hip
q4_des = pi/2 + knee_amp * sin(omega * t + phase_offset); % Right knee

% Combine results into pts_foot matrix (for one period)
pts_foot = [q1_des; q2_des; q3_des; q4_des; t];

%to HERE!

%the rest is just to debug the points.

% Display total trajectory time
total_trajectory_time = t(end) - t(1);
fprintf('Total Trajectory Time (one period): %.2f seconds\n', total_trajectory_time);

% Display pts_foot for inspection (optional)
disp('pts_foot =');
disp(pts_foot);


% Animation setup (optional)
figure;
hold on;
xlabel('X Position');
ylabel('Y Position');
title('Foot Trajectory Animation (One Period)');
axis equal;
grid on;
xlim([-pi/4 - hip_amp, pi/4 + hip_amp]);  % Set x-axis limits
ylim([pi/2 - knee_amp, pi/2 + knee_amp]); % Set y-axis limits

% Initialize plot elements
left_foot_plot = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
right_foot_plot = plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

% Animate the points (optional)
for i = 1:num_points
    % Update left foot position (x, y) based on q1_des, q2_des
    left_x = q1_des(i);
    left_y = q2_des(i);
    
    % Update right foot position (x, y) based on q3_des, q4_des
    right_x = q3_des(i);
    right_y = q4_des(i);
    
    % Update plots
    set(left_foot_plot, 'XData', left_x, 'YData', left_y);
    set(right_foot_plot, 'XData', right_x, 'YData', right_y);
    
    % Pause to create animation effect
    pause(0.05);
end

hold off;
