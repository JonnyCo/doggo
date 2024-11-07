function keypoints = keypoints_leg(in1,in2)
% use leg lengths to calculate key points for plotting

l_AC = in2(3,:);
l_DE = in2(4,:);
l_OA = in2(1,:);
l_OB = in2(2,:);
th1  = -in1(1,:); % negative due to direction motors are mounted
th2  = -in1(2,:); % negative due to direction motors are mounted

t2 = sin(th1);
t3 = l_OA.*t2;
t4 = th1+th2;
t5 = sin(t4);
t6 = l_AC.*t5;
t7 = l_OB.*t2;
t8 = cos(th1);
t9 = cos(t4);



keypoints = reshape([t3,-l_OA.*t8,t7,-l_OB.*t8,t3+t6,-l_AC.*t9-l_OA.*t8,t6+t7,-l_AC.*t9-l_OB.*t8,t6+t7+l_DE.*t2,-l_AC.*t9-l_DE.*t8-l_OB.*t8],[2, 5]);
