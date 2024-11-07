function inertia_ellipse = inertia_ellipse_leg(in1,in2)
% use leg parameters to calculate operational space inertia, represent as an ellipse

l_AC = in2(3,:);
l_DE = in2(4,:);
l_OA = in2(1,:);
l_OB = in2(2,:);

m1   = in2(5,:);
m2   = in2(6,:);
m3   = in2(7,:);
m4   = in2(8,:);

I1   = in2(9,:);
I2   = in2(10,:);
I3   = in2(11,:);
I4   = in2(12,:);

Ir   = in2(13,:);
Nmot = in2(14,:);

l_O_m1 = in2(15,:);
l_B_m2 = in2(16,:);
l_A_m3 = in2(17,:);
l_C_m4 = in2(18,:);

th1  = -in1(1,:); % negative due to direction motors are mounted
th2  = -in1(2,:); % negative due to direction motors are mounted

% calculate Jacobian
J = [ l_AC*cos(th1 + th2) + l_DE*cos(th1) + l_OB*cos(th1), l_AC*cos(th1 + th2);
      l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1), l_AC*sin(th1 + th2)];
  
% calculate joint space mass matrix
M11 = I1 + I2 + I3 + I4 + Ir + Ir*Nmot^2 + l_AC^2*m4 + l_A_m3^2*m3 + l_B_m2^2*m2 + l_C_m4^2*m4 + l_OA^2*m3 + l_OB^2*m2 + l_OA^2*m4 + l_O_m1^2*m1 + 2*l_C_m4*l_OA*m4 + 2*l_AC*l_C_m4*m4*cos(th2) + 2*l_AC*l_OA*m4*cos(th2) + 2*l_A_m3*l_OA*m3*cos(th2) + 2*l_B_m2*l_OB*m2*cos(th2);
M12 = I2 + I3 + l_AC^2*m4 + l_A_m3^2*m3 + l_B_m2^2*m2 + Ir*Nmot + l_AC*l_C_m4*m4*cos(th2) + l_AC*l_OA*m4*cos(th2) + l_A_m3*l_OA*m3*cos(th2) + l_B_m2*l_OB*m2*cos(th2);
M22 = Ir*Nmot^2 + m4*l_AC^2 + m3*l_A_m3^2 + m2*l_B_m2^2 + I2 + I3;
M = [ M11, M12; M12, M22];

% calculate the operational space mass matrix
LL = inv( (J/M)*J' );

% get eigenvectors and corresponding values
[V,D] = eig(LL); % columns of V are eigenvectors, diagonal elements of D are eigenvalues

% using parametric equations of ellipse, calculate ellipse points relative to foot position
th_ellipse = -atan2(V(1,1),V(2,1)); % angle between first eigenvector and positive x axis
l_x = 0.01*(1/D(1,1)); % TODO: better way to implement scaling of the ellipse?
l_y = 0.01*(1/D(2,2)); 
ii = linspace(0, 2*pi, 100);
xpts = (l_x*cos(ii))*cos(th_ellipse) - (l_y*sin(ii))*sin(th_ellipse);
ypts = (l_x*cos(ii))*sin(th_ellipse) + (l_y*sin(ii))*cos(th_ellipse);

inertia_ellipse = [xpts;ypts];
