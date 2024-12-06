% Updated to send over cycles
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

cycles = 10;

%  [traj_time, q1_des_1, q2_des_1, q3_des_1, q4_des_1,  q1_des_2, q2_des_2, q3_des_2, q4_des_2, q1_des_3, q2_des_3, q3_des_3, q4_des_3, q1_des_4, q2_des_4, q3_des_4, q4_des_4, q1_des_5, q2_des_5, q3_des_5, q4_des_5, K_h, K_n, D_h, D_ k]     
%pts_foot = [0.4049513091773823 -0.27780461189194694	0.353418001876763	-0.05152539598804029	0.3390406935772966	-0.24709808370152408	0.09625072439682969	0.04075972034615549	0.023395003224749522	0.38659513136638435	-0.1094310323956641	0.03420424701450686	-0.340019683770868	-0.39634316600584296	0.197101041057138	0.03176614252066534	-0.09251215537969015	0.2870128934422118	0.17663266920127196	-0.10836684324938795];
%pts_foot = [0.17304984673393564	0.010553837153948864	0.45797218807899764	0.4698650886790107	0.10309168146361247	0.04329158783281917	0.3718055369297545	0.46715681054027725	0.4771846392059128	0.17015777964909978	0.11732075142425878	0.06641355162323864	0.23074656154744902	0.33174650373260417	0.4540213787932398	0.44455960922947224	0.3111595699275232	0.26403981571850166	0.10424906284044654	0.1648176819160193 ]; % Genomic 0.2m and 0.3kg
%pts_foot = pts_foot .* [1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1];
%pts_foot = pts_foot' ;

%great great pts_foot = [0.000000, -1.178097, 0.523599, -1.727876, 0.373479, -1.449446, 0.374207, -1.619337, 0.230823, -1.888497, 0.431270, -1.443716, -0.230823, -1.888497, 0.615928, -1.443716, -0.373479, -1.449446, 0.672990, -1.619337];

pts_foot = [0.785398, -1.178097, 0.523599, -1.745329, 1.158877, -1.449446, 0.374207, -1.624730, 1.016221, -1.888497, 0.431270, -1.429596, 0.554575, -1.888497, 0.615928, -1.429596, 0.411919, -1.449446, 0.672990, -1.624730];

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = -pi/2; 

angle3_init = 0;
angle4_init = -pi/2; 

% Total experiment time is buffer,trajectory,buffer
traj_time         = .50; % can also use this as cycle time
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 3;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_h = 2.5; % 1 used to work % 2.5 = max static
gains.K_k = 2.5;

gains.D_h = 0.08;  % 0.15;% 0.015 used to work
gains.D_k = 0.08;


% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_maxF   = .8;
duty_maxB   = .8;

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

torques_squared = output_data(:,38);
tauq1 = output_data(:,39);
tauq2 = output_data(:,40);
tauq3 = output_data(:,41);
tauq4 = output_data(:,42);

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

figure(5); clf;
plot(t,torques_squared,'r-');
xlabel('Time (s)'); ylabel('Torques^2'); title("Torques^2");

figure(6); clf;
hold on; % Allows multiple plots on the same graph
plot(t, tauq1, 'LineWidth', 1.5, 'DisplayName', 'Torque q1'); % Torque q1
plot(t, tauq2, 'LineWidth', 1.5, 'DisplayName', 'Torque q2'); % Torque q2
plot(t, tauq3, 'LineWidth', 1.5, 'DisplayName', 'Torque q3'); % Torque q3
plot(t, tauq4, 'LineWidth', 1.5, 'DisplayName', 'Torque q4'); % Torque q4
hold off; % Release the graph for future plotting

xlabel('Time (s)');
ylabel('Torques');
title('Torques for q1, q2, q3, q4');
legend('show'); % Automatically display the names from 'DisplayName'
grid on; % Add grid lines for better visualization

