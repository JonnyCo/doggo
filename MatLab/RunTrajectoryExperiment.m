function output_data = RunTrajectoryExperiment(cycles, angle1_init, angle2_init, angle3_init, angle4_init, pts_foot, traj_time, pre_buffer_time, post_buffer_time, gains, duty_maxF, duty_maxB)
    
    % Figure for plotting motor data
    figure(1);  clf;       
    a1 = subplot(8,2,1);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)'); title("Front Leg");
    
    a2 = subplot(8,2,5);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)'); title("Front Leg");
    
    a3 = subplot(8,2,9);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)'); title("Front Leg");
    hold on;
    subplot(8,2,9);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(8,2,13);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Cycle 1'); title("Front Leg");

    a5 = subplot(8,2,2);
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)'); title("Front Leg");
    
    a6 = subplot(8,2,6);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity 2 (rad/s)'); title("Front Leg");
    
    a7 = subplot(8,2,10);
    h23 = plot([0],[0]);
    h23.XData = []; h23.YData = [];
    ylabel('Current 2 (A)'); title("Front Leg");
    hold on;
    subplot(8,2,10);
    h24 = plot([0],[0],'r');
    h24.XData = []; h24.YData = [];
    hold off;
    
    a8 = subplot(8,2,14);
    h25 = plot([0],[0]);
    h25.XData = []; h25.YData = [];
    ylabel('Duty Cycle 2'); title("Front Leg");
    
    %Leg 2      
    a9 = subplot(8,2,3);
    h31 = plot([0],[0]);
    h31.XData = []; h31.YData = [];
    ylabel('Angle 3 (rad)'); title("Back Leg");
    
    a10 = subplot(8,2,7);
    h32 = plot([0],[0]);
    h32.XData = []; h32.YData = [];
    ylabel('Velocity 3 (rad/s)'); title("Back Leg");
    
    a11 = subplot(8,2,11);
    h33 = plot([0],[0]);
    h33.XData = []; h33.YData = [];
    ylabel('Current 3 (A)'); title("Back Leg");
    hold on;
    subplot(8,2,11);
    h34 = plot([0],[0],'r');
    h34.XData = []; h34.YData = [];
    hold off;
    
    a12 = subplot(8,2,15);
    h35 = plot([0],[0]);
    h35.XData = []; h35.YData = [];
    ylabel('Duty Cycle 3'); title("Back Leg");

    a13 = subplot(8,2,4);
    h41 = plot([0],[0]);
    h41.XData = []; h41.YData = [];
    ylabel('Angle 4 (rad)'); title("Back Leg");
    
    a14 = subplot(8,2,8);
    h42 = plot([0],[0]);
    h42.XData = []; h42.YData = [];
    ylabel('Velocity 4 (rad/s)'); title("Back Leg");
    
    a15 = subplot(8,2,12);
    h43 = plot([0],[0]);
    h43.XData = []; h43.YData = [];
    ylabel('Current 4 (A)'); title("Back Leg");
    hold on;
    subplot(8,2,12);
    h44 = plot([0],[0],'r');
    h44.XData = []; h44.YData = [];
    hold off;
    
    a16 = subplot(8,2,16);
    h45 = plot([0],[0]);
    h45.XData = []; h45.YData = [];
    ylabel('Duty Cycle 4'); title("Back Leg");

    % Figure for plotting state of the leg
    figure(2)
    clf
    hold on
    axis equal
    axis([-.35 .35 -.35 .2]);
   
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    h_ellip = plot([0],[0],'g','LineWidth',1.5);

    h_OB2 = plot([0],[0],'LineWidth',2);
    h_AC2 = plot([0],[0],'LineWidth',2);
    h_BD2 = plot([0],[0],'LineWidth',2);
    h_CE2 = plot([0],[0],'LineWidth',2);
    h_ellip2 = plot([0],[0],'g','LineWidth',1.5);
    
    h_foot= plot([0],[0],'Color',[0.7,0.7,0.7]);
    h_des = plot([0],[0],'--','Color',[0.5,0.5,0.5]);
    h_des.XData=[];
    h_des.YData=[];
    h_foot.XData=[];
    h_foot.YData=[];

    h_foot2 = plot([0],[0],'Color',[0.7,0.7,0.7]);
    h_des2 = plot([0],[0],'--','Color',[0.5,0.5,0.5]);
    h_des2.XData=[];
    h_des2.YData=[];
    h_foot2.XData=[];
    h_foot2.YData=[];
    
    % Define leg length parameters
    m1 =.0393 + .2;         m2 =.0368; 
    m3 = .00783;            m4 = .0155;
    I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
    I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
    l_OA=.011;              l_OB=.042; 
    l_AC=.096;              l_DE=.091;
    l_O_m1=0.032;           l_B_m2=0.0344; 
    l_A_m3=0.0622;          l_C_m4=0.0610;
    Nmot = 18.75;
    Ir = 0.0035/Nmot^2;

    p   = [l_OA l_OB l_AC l_DE];
    p   = [p m1 m2 m3 m4 I1 I2 I3 I4 Ir Nmot l_O_m1 l_B_m2 l_A_m3 l_C_m4]';
    
    offset = 0.285;
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time

        pos1 = new_data(:,2);       % position
        vel1 = new_data(:,3);       % velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command
        
        pos2 = new_data(:,7);       % position
        vel2 = new_data(:,8);       % velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command

        pos3 = new_data(:,12);       % position
        vel3 = new_data(:,13);       % velocity
        cur3 = new_data(:,14);       % current
        dcur3 = new_data(:,15);      % desired current
        duty3 = new_data(:,16);      % command
        
        pos4 = new_data(:,17);       % position
        vel4 = new_data(:,18);       % velocity
        cur4 = new_data(:,19);       % current
        dcur4 = new_data(:,20);     % desired current
        duty4 = new_data(:,21);     % command
        
        xF = -new_data(:,22);         % actual foot position (negative due to direction motors are mounted)
        yF = new_data(:,23);         % actual foot position
        xdesF = -new_data(:,26);      % desired foot position (negative due to direction motors are mounted)
        ydesF = new_data(:,27);      % desired foot position        

        xB = -new_data(:,30);        
        yB = new_data(:,31);        
        xdesB = -new_data(:,34);      
        ydesB = new_data(:,35); 
        
        N = length(pos1);
        
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = -pos1; % switch sign on all plotted values due to direction motors are mounted
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = -vel1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = -cur1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = -dcur1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = -duty1;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = -pos2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = -vel2;
        h23.XData(end+1:end+N) = t;   
        h23.YData(end+1:end+N) = -cur2;
        h24.XData(end+1:end+N) = t;   
        h24.YData(end+1:end+N) = -dcur2;
        h25.XData(end+1:end+N) = t;   
        h25.YData(end+1:end+N) = -duty2;

        h31.XData(end+1:end+N) = t;   
        h31.YData(end+1:end+N) = -pos3;
        h32.XData(end+1:end+N) = t;   
        h32.YData(end+1:end+N) = -vel3;
        h33.XData(end+1:end+N) = t;   
        h33.YData(end+1:end+N) = -cur3;
        h34.XData(end+1:end+N) = t;   
        h34.YData(end+1:end+N) = -dcur3;
        h35.XData(end+1:end+N) = t;   
        h35.YData(end+1:end+N) = -duty3;

        h41.XData(end+1:end+N) = t;   
        h41.YData(end+1:end+N) = -pos4;
        h42.XData(end+1:end+N) = t;   
        h42.YData(end+1:end+N) = -vel4;
        h43.XData(end+1:end+N) = t;   
        h43.YData(end+1:end+N) = -cur4;
        h44.XData(end+1:end+N) = t;   
        h44.YData(end+1:end+N) = -dcur4;
        h45.XData(end+1:end+N) = t;   
        h45.YData(end+1:end+N) = -duty4;
        
        % Calculate leg state and update plots
        zF = [pos1(end) pos2(end) vel1(end) vel2(end)]';
        keypointsF = keypoints_leg(zF,p);
        inertia_ellipseF = inertia_ellipse_leg(zF,p);

        zB = [pos3(end) pos4(end) vel3(end) vel4(end)]';
        keypointsB = keypoints_leg(zB,p);
        inertia_ellipseB = inertia_ellipse_leg(zB,p);
        
        % TODO: could also plot Jacobian, control force vector here?
        
        rA = keypointsF(:,1); 
        rB = keypointsF(:,2);
        rC = keypointsF(:,3);
        rD = keypointsF(:,4);
        rE = keypointsF(:,5);

        rA2 = keypointsB(:,1); 
        rB2 = keypointsB(:,2);
        rC2 = keypointsB(:,3);
        rD2 = keypointsB(:,4);
        rE2 = keypointsB(:,5);

        set(h_OB,'XData',[offset/2 rB(1)+offset/2],'YData',[0 rB(2)]);
        set(h_AC,'XData',[rA(1)+offset/2 rC(1)+offset/2],'YData',[rA(2) rC(2)]);
        set(h_BD,'XData',[rB(1)+offset/2 rD(1)+offset/2],'YData',[rB(2) rD(2)]);
        set(h_CE,'XData',[rC(1)+offset/2 rE(1)+offset/2],'YData',[rC(2) rE(2)]);

        set(h_OB2,'XData',[-offset/2 rB2(1)-offset/2],'YData',[0 rB2(2)]);
        set(h_AC2,'XData',[rA2(1)-offset/2 rC2(1)-offset/2],'YData',[rA2(2) rC2(2)]);
        set(h_BD2,'XData',[rB2(1)-offset/2 rD2(1)-offset/2],'YData',[rB2(2) rD2(2)]);
        set(h_CE2,'XData',[rC2(1)-offset/2 rE2(1)-offset/2],'YData',[rC2(2) rE2(2)]);
        
        ellipse_xF = inertia_ellipseF(1,:) + rE(1)+offset/2;
        ellipse_yF = inertia_ellipseF(2,:) + rE(2);
        set(h_ellip,'XData',ellipse_xF,'YData',ellipse_yF);

        ellipse_xB = inertia_ellipseB(1,:) + rE(1)-offset/2;
        ellipse_yB = inertia_ellipseB(2,:) + rE(2);
        set(h_ellip2,'XData',ellipse_xB,'YData',ellipse_yB);
        
        h_foot.XData(end+1:end+N) = xF;
        h_foot.YData(end+1:end+N) = yF;
        h_des.XData(end+1:end+N) = xdesF;
        h_des.YData(end+1:end+N) = ydesF;

        h_foot2.XData(end+1:end+N) = xB;
        h_foot2.YData(end+1:end+N) = yB;
        h_des2.XData(end+1:end+N) = xdesB;
        h_des2.YData(end+1:end+N) = ydesB;
        
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
    start_period                = pre_buffer_time;    % In seconds 
    end_period                  = post_buffer_time;   % In seconds
    
    K_h                     = gains.K_h; % Stiffness hip
    K_k                     = gains.K_k; % Stiffness knee


    D_h                     = gains.D_h; % Damping hip
    D_k                     = gains.D_k; % Damping knee

    
    % Specify inputs
    input = [cycles];
    input = [input start_period traj_time end_period];
    input = [input angle1_init angle2_init angle3_init angle4_init];
    input = [input K_h K_k D_h D_k]; % hip and knee params
    input = [input duty_maxF duty_maxB];
    input = [input pts_foot(:)']; % final size of input should be 28x1 14 + 20 = 34
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 37;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2 a3 a4],'x')
    
end
