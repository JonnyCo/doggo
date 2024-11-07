% This is the main MATLAB script for Lab 5.
%
% You will need to modify the Mbed code and this script, but should not need to make any other changes.
%
%% SET YOUR INPUTS HERE

% Bezier curve control points
const_point = [-0.05; -0.165]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
%pts_foot = repmat(const_point,1,8);
       
% YOUR BEZIER PTS HERE
%pts_foot = [0.1265    0.1265    0.1265    0.1313   -0.1926   -0.1732   -0.1732   -0.1732; -0.1760   -0.1760   -0.1760   -0.0524   -0.0048   -0.1351   -0.1351   -0.1351];
pts_foot = [-0.10 -0.10 -0.05   0.05 0.15 0.15  -0.05 -0.10;
            -0.15 -0.16 -0.165 -0.17 0.14 -0.13 -0.13 -0.15];
% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = -pi/2; 

angle3_init = 0;
angle4_init = -pi/2; 

% Total experiment time is buffer,trajectory,buffer
traj_time         = 1;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 3;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xxF = 250.0;
gains.K_yyF = 250.0;
gains.K_xyF = 0.0;

gains.K_xxB = 250.0;
gains.K_yyB = 250.0;
gains.K_xyB = 0;

gains.D_xxF = 15.0;
gains.D_yyF = 15.0;
gains.D_xyF = 0;

gains.D_xxB = 20.0;
gains.D_yyB = 20.0;
gains.D_xyB = 0;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_maxF   = 0.4;
duty_maxB   = 0.4;

%% Run Experiment
[output_data] = RunTrajectoryExperiment(angle1_init, angle2_init, angle3_init, angle4_init, pts_foot,...
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