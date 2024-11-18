% Updated to send over cycles
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

cycles = 5;

%  [traj_time, q1_des_1, q2_des_1, q3_des_1, q4_des_1,  q1_des_2, q2_des_2, q3_des_2, q4_des_2, q1_des_3, q2_des_3, q3_des_3, q4_des_3, q1_des_4, q2_des_4, q3_des_4, q4_des_4, q1_des_5, q2_des_5, q3_des_5, q4_des_5, K_h, K_n, D_h, D_ k]     
pts_foot = [0.785398, 0.523599, 0.785398, 0.000000, 1.034384, 0.342700, 0.536412, 0.180899, 0.939280, 0.049999, 0.631516, 0.473600, 0.631516, 0.049999, 0.939280, 0.473600, 0.536412, 0.342700, 1.034384, 0.180899];
% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = -pi/2; 

angle3_init = 0;
angle4_init = -pi/2; 

% Total experiment time is buffer,trajectory,buffer
traj_time         = 1; % can also use this as cycle time
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 3;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_h = 1;
gains.K_k = 1;

gains.D_h = .015;
gains.D_k = .015;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_maxF   = 0.8;
duty_maxB   = 0.8;

%% Run Experiment
[output_data] = RunTrajectoryExperiment(cycles, angle1_init, angle2_init, angle3_init, angle4_init, pts_foot,...
                                        traj_time, pre_buffer_time, post_buffer_time,...
                                        gains, duty_maxF, duty_maxB);

%% Extract data
t = output_data(:,1);

xF = -output_data(:,21); % actual foot position in X (negative due to direction motors are mounted)
yF = output_data(:,22); % actual foot position in Y
   
xdesF = -output_data(:,25); % desired foot position in X (negative due to direction motors are mounted)
ydesF = output_data(:,26); % desired foot position in Y

xB = -output_data(:,29);
yB = output_data(:,30);

xdesB = -output_data(:,33);
ydesB = output_data(:,34);

%% Plot foot vs desired
figure(3); clf;
subplot(321); hold on
plot(t,xdesF,'r-'); plot(t,xF);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'}); title("Front Leg");

subplot(323); hold on
plot(t,ydesF,'r-'); plot(t,yF);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'}); title("Front Leg"); 

subplot(325); hold on
plot(xdesF,ydesF,'r-'); plot(xF,yF,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'}); title("Front Leg");

subplot(322); hold on
plot(t,xdesB,'r-'); plot(t,xB);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'}); title("Back Leg");

subplot(324); hold on
plot(t,ydesB,'r-'); plot(t,yB);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'}); title("Back Leg");

subplot(326); hold on
plot(xdesB,ydesB,'r-'); plot(xB,yB,'k'); title("Back Leg");
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});
