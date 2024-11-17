% Parameters for joint-space trajectory
cycle_period = 1.0; % Total period of one cycle in seconds
num_points = 5; % Generate exactly 5 sets of points
omega = 2 * pi / cycle_period; % Angular frequency (rad/s)

% Amplitude settings for joint oscillations (in radians)
hip_amp_front = 0; %pi / 12; % Amplitude of front hip oscillation
knee_amp_front = 0; %pi / 24; % Amplitude of front knee oscillation
hip_amp_rear = 0; %pi / 12; % Amplitude of rear hip oscillation
knee_amp_rear = 0; %pi / 24; % Amplitude of rear knee oscillation

% Rotation offsets (in radians)
hip_offset_front = -pi/4; % Offset for front hip
knee_offset_front = 0; % Offset for front knee
hip_offset_rear = -pi/4; % Offset for rear hip
knee_offset_rear = 0; % Offset for rear knee

% Initialize arrays to store trajectory points
q1_des = zeros(1, num_points); % Front hip
q2_des = zeros(1, num_points); % Front knee
q3_des = zeros(1, num_points); % Rear hip
q4_des = zeros(1, num_points); % Rear knee
t_q = linspace(0, cycle_period - cycle_period/num_points, num_points); % Time points for trajectory (adjusted)

% Generate sinusoidal trajectories for front leg (hip: q1, knee: q2)
for i = 1:num_points
    t = t_q(i);
    q1_des(i) = hip_offset_front + hip_amp_front * sin(omega * t); % Centered around hip_offset_front
    q2_des(i) = knee_offset_front + knee_amp_front * cos(omega * t); % Centered around knee_offset_front
end

% Generate sinusoidal trajectories for rear leg (hip: q3, knee: q4), with phase shift
for i = 1:num_points
    t = t_q(i);
    q3_des(i) = hip_offset_rear + hip_amp_rear * sin(omega * t + pi); % Rear hip with offset
    q4_des(i) = knee_offset_rear + knee_amp_rear * cos(omega * t + pi); % Rear knee with offset
end

% Control parameters (example values)
K_L = 15.0;
K_R = 15.0;
D_L = 0.5;
D_R = 0.5;

% Format the output as input_params in the correct order
input_params = [cycle_period];
for i = 1:num_points
    input_params = [input_params, q1_des(i), q2_des(i), q3_des(i), q4_des(i)];
end
input_params = [input_params, K_L, K_R, D_L, D_R];

% Display the result
fprintf('input_params = [');
fprintf('%f, ', input_params(1:end-1));
fprintf('%f];\n', input_params(end));

% Plotting
figure;
hold on;
title('Elliptical Trajectories of Front and Rear Legs with Rotation Offsets');
xlabel('Hip Angle (rad)');
ylabel('Knee Angle (rad)');
grid on;

% Plot front leg trajectory
plot(q1_des, q2_des, '-o', 'DisplayName', 'Front Leg Trajectory');
text(q1_des, q2_des, arrayfun(@(i) sprintf('P%d', i), 1:num_points, 'UniformOutput', false), 'VerticalAlignment', 'bottom');

% Plot rear leg trajectory
plot(q3_des, q4_des, '-o', 'DisplayName', 'Rear Leg Trajectory');
text(q3_des, q4_des, arrayfun(@(i) sprintf('P%d', i), 1:num_points, 'UniformOutput', false), 'VerticalAlignment', 'bottom');

legend;
hold off;
