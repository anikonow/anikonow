
%% intelroboticshw4 %%


clc
clear

syms L d t theta1 theta2 theta3 theta4 theta5 theta6 theta7;
%t1 = 0;
% t2 = 0;
% t3 = pi/2;
% t4 = -pi/2;
% 
% f=[-10 10 0]';
% m=[0 0 -10]';
% 
% s1= [cos(t1) -sin(t1) 0; sin(t1) cos(t1) 0; 0 0 1];
% s2= [cos(t1+t2) -sin(t1+t2) 0; sin(t1+t2) cos(t1+t2) 0; 0 1 1];
% s3= [cos(t1+t2+t3) -sin(t1+t2+t3) 0; sin(t1+t2+t3) cos(t1+t2+t3) 0; 0 1 1];
% s4= [cos(t1+t2+t3+t4) -sin(t1+t2+t3+t4) 0; sin(t1+t2+t3+t4) cos(t1+t2+t3+t4) 0; 0 1 1];
% 
% 
% m1 = s1*m;
% m2 = s2*m;
% m3 = s3*m;
% m4 = s4*m;
% 
% disp(m1)
% disp(m2)
% disp(m3)
% disp(m4)
% 
% SB= exp(s1) + exp(s2) + exp(s3) + exp(s4);
% rank(SB)

% R=[1 0 L; 0 1 L; 0 0 1];
% B=[cos(t1) -sin(t1) 0; sin(t1) cos(t1) 0; 0 -d 1];

% T_trans = [1, 0, 0, L*(1-cos(theta));
%            0, 1, 0, L*(1-sin(theta));
%            0, 0, 1, 0;
%            0, 0, 0, 1];
% 
% T_rot = [cos(theta), -sin(theta), 0, 0;
%          sin(theta), cos(theta), 0, 0;
%          0, 0, 1, 0;
%          0, 0, 0, 1];
% 
% T = T_trans * T_rot;
% 
% T_inv = inv(T);
% 
% T_sym = T_trans * [cos(t), -sin(t), 0, 0;
%                    sin(t), cos(t), 0, 0;
%                    0, 0, 1, 0;
%                    0, 0, 0, 1]; 


%% Question 4
% T0_1= [cos((pi/2)) -sin((pi/2)) 0 0; sin((pi/2)) cos((pi/2)) 0 0; 0 0 1 0; 0 0 55 1];
% T1_2= [cos((pi/2)) 0 sin((pi/2)) 0; 0 1 0 0; -sin((pi/2)) 0 cos((pi/2)) 0; 0 0 0 1];
% T2_3= [1 0 0 0; 0 cos((pi/2)) -sin((pi/2)) 0; 0 sin(pi/2) cos((pi/2)) 0; 0 0 0 1];
% 
% T3_4= [cos((pi/2)) 0 sin((pi/2)) 0; 0 1 0 0; -sin(pi/2) 0 cos(pi/2) 0; 0 0 30 1];
% 
% T4_5= [cos(pi/2) -sin(pi/2) 0 0; sin(pi/2) cos(pi/2) 0 0; 0 0 1 0; 0 0 18 1];
% T5_6= [cos(pi/2) 0 sin(pi/2) 0; 0 1 0 0; -sin(pi/2) 0 cos(pi/2) 0; 0 0 0 1];
% T6_7= [1 0 0 0; 0 cos(pi/2) -sin(pi/2) 0; 0 sin(pi/2) cos(pi/2) 0; 0 0 0 1];
% 
% 
% 
% Jtheta= [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 1 0 -55 0;1 0 0 0 -85 0; 0 1 0 85 0 0; 0 0 1 0 0 0];
% A=Jtheta*Jtheta';
% AA=eigs(A);
% 
% disp(AA(1:3,:))
% 
% load("hw2_xyz.txt")
% 
% for i = 1:835
%   load(i,:) = hw2_xyz(i+1,:) - hw2_xyz(i,:);
% end
% figure;
% subplot(3, 1, 1);
% plot(1:835, load(:, 1), 'b');
% xlabel('time');
% ylabel('velo');
% title('x velo');
% 
% subplot(3, 1, 2);
% plot(1:835, load(:, 2), 'g');
% xlabel('time');
% ylabel('velo');
% title('y velo');
% 
% subplot(3, 1, 3);
% plot(1:835, load(:, 3), 'r');
% xlabel('time');
% ylabel('velo');
% title('z velo');
% 
% 
% %%%dlmwrite('hw3_xyz.txt', load)
% %% Question 5
% 
% theta_data= readtable("HW2_Theta_data.txt");
% t1=theta_data{:,1};
% t2=theta_data{:,2};
% t3=theta_data{:,3};
% t4=theta_data{:,4};
% t5=theta_data{:,5};
% t6=theta_data{:,6};
% t7=theta_data{:,7};
% 
% L1= 55; %j1 to j2 cm
% L2= 30; %j2 to j3 cm
% L3= 18; %j3to tip cm
% 
% i = 1;
% for i= 1:836
% T0_1= [cos(t1(i,:)) -sin(t1(i,:)) 0 0; sin(t1(i,:)) cos(t1(i,:)) 0 0; 0 0 1 0; 0 0 55 1];
% T1_2= [cos(t2(i,:)) 0 sin(t2(i,:)) 0; 0 1 0 0; -sin(t2(i,:)) 0 cos(t2(i,:)) 0; 0 0 0 1];
% T2_3= [1 0 0 0; 0 cos(t3(i,:)) -sin(t3(i,:)) 0; 0 sin(t3(i,:)) cos(t3(i,:)) 0; 0 0 0 1];
% 
% T3_4= [cos(t4(i,:)) 0 sin(t4(i,:)) 0; 0 1 0 0; -sin(t4(i,:)) 0 cos(t4(i,:)) 0; 0 0 30 1];
% 
% T4_5= [cos(t5(i,:)) -sin(t5(i,:)) 0 0; sin(t5(i,:)) cos(t5(i,:)) 0 0; 0 0 1 0; 0 0 18 1];
% T5_6= [cos(t6(i,:)) 0 sin(t6(i,:)) 0; 0 1 0 0; -sin(t6(i,:)) 0 cos(t6(i,:)) 0; 0 0 0 1];
% T6_7= [1 0 0 0; 0 cos(t7(i,:)) -sin(t7(i,:)) 0; 0 sin(t7(i,:)) cos(t7(i,:)) 0; 0 0 0 1];
% 
% T0_7=T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7;
% T0_7= T0_7';
% 
% end_effector_positions(i, :) = T0_7(1:3, 4)';
% end
% 
% 
% figure;
% plot3(end_effector_positions(:, 1), end_effector_positions(:, 2), end_effector_positions(:, 3), 'b', 'LineWidth', 2);
% hold on;
% plot3(end_effector_positions(1, 1), end_effector_positions(1, 2), end_effector_positions(1, 3), 'ro', 'MarkerSize', 10); 
% plot3(end_effector_positions(end, 1), end_effector_positions(end, 2), end_effector_positions(end, 3), 'go', 'MarkerSize', 10); 
% grid on;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('End-Effector Position');
% legend('End-Effector Path', 'Initial Position', 'Final Position');
% view(3);

% %dlmwrite('hw2_xyz.txt', end_effector_positions)


%% hw4 p4 part A
T0_1= [cos(.1) -sin(.1) 0 0; sin(.1) cos(.1) 0 0; 0 0 1 0; 0 0 .55 1];
T1_2= [cos(.2) 0 sin(.2) 0; 0 1 0 0; -sin(.2) 0 cos(.2) 0; 0 0 0 1];
T2_3= [1 0 0 0; 0 cos(.3) -sin(.3) 0; 0 sin(.3) cos((.3)) 0; 0 0 0 1];

T3_4= [cos(.4) 0 sin(.4) 0; 0 1 0 0; -sin(.4) 0 cos(.4) 0; 0 0 .30 1];

T4_5= [cos(.5) -sin(.5) 0 0; sin(.5) cos(.5) 0 0; 0 0 1 0; 0 0 .18 1];
T5_6= [cos(.6) 0 sin(.6) 0; 0 1 0 0; -sin(.6) 0 cos(.6) 0; 0 0 0 1];
T6_7= [1 0 0 0; 0 cos(.7) -sin(.7) 0; 0 sin(.7) cos(.7) 0; 0 0 0 1];

T0_7 = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7;
T0_7=T0_7'


t1=.1;
t2=.2;
t3=.3;
t4=.4;
t5=.5;
t6=.6;
t7=.7;



s1= [cos(t1) -sin(t1) 0; sin(t1) cos(t1) 0; 0 0 1];
s2= [cos(t1+t2) -sin(t1+t2) 0; sin(t1+t2) cos(t1+t2) 0; 0 1 1];
s3= [cos(t1+t2+t3) -sin(t1+t2+t3) 0; sin(t1+t2+t3) cos(t1+t2+t3) 0; 0 1 1];
s4= [cos(t1+t2+t3+t4) -sin(t1+t2+t3+t4) 0; sin(t1+t2+t3+t4) cos(t1+t2+t3+t4) 0; 0 1 1];
s5= [cos(t1+t2+t3+t4+t5) -sin(t1+t2+t3+t4+t5) 0; sin(t1+t2+t3+t4+t5) cos(t1+t2+t3+t4+t5) 0; 0 1 1];
s6= [cos(t1+t2+t3+t4+t5+t6) -sin(t1+t2+t3+t4+t5+t6) 0; sin(t1+t2+t3+t4+t5+t6) cos(t1+t2+t3+t4+t5+t6) 0; 0 1 1];
s7= [cos(t1+t2+t3+t4+t5+t6+t7) -sin(t1+t2+t3+t4+t5+t6+t7) 0; sin(t1+t2+t3+t4+t5+t6+t7) cos(t1+t2+t3+t4+t5+t6+t7) 0; 0 1 1];
thetadot= [1 1 1 1 1 1 1 1];

Jb=eye(3)*expm(s1)*expm(s2)*expm(s3)*expm(s4)*expm(s5)*expm(s6)*expm(s7);
Jb=Jb';

Jv=[Jb zeros(3); zeros(3) Jb]

%% hw 4 part b %%
T0_7;
Txd = [ -0.7954 -0.4634 -0.3906 0.46320; -0.0277 -0.6160 0.7873 1.16402; -0.6054 0.6370 0.4771 2.22058;0 0 0 1 ]; 


R1 = T0_7(1:3, 1:3);
p1 = T0_7(1:3, 4);


R2 = Txd(1:3, 1:3);
p2 = Txd(1:3, 4);


R = R1' * R2;
p = p2 - p1;
w_hat = logm(R);

theta = acos((trace(R) - 1) / 2);  
w = theta / (2 * sin(theta)) * [w_hat(3,2); w_hat(1,3); w_hat(2,1)];
V = [w; p]

%% hw 4 part c %%

t1=.1;
t2=.2;
t3=.3;
t4=.4;
t5=.5;
t6=.6;
t7=.7;

T0= [0.2463	0.6428	-0.7252	-0.6699; 0.2389	0.6849	0.6882	0.6337;0.9392 -0.3429 0.01510	0.3113;0	0	0	1];
Txd = [ -0.7954 -0.4634 -0.3906 0.46320; -0.0277 -0.6160 0.7873 1.16402; -0.6054 0.6370 0.4771 2.22058;0 0 0 1 ]; 
Txd2= [-0.3741 -0.9211 -0.1082 0.49796; 0.8848 -0.3894 0.2560 0.98500; -0.2779 0.0000 0.9606 2.34041; 0 0 0 1];

s1= [cos(t1) -sin(t1) 0; sin(t1) cos(t1) 0; 0 0 1];
s2= [cos(t1+t2) -sin(t1+t2) 0; sin(t1+t2) cos(t1+t2) 0; 0 1 1];
s3= [cos(t1+t2+t3) -sin(t1+t2+t3) 0; sin(t1+t2+t3) cos(t1+t2+t3) 0; 0 1 1];
s4= [cos(t1+t2+t3+t4) -sin(t1+t2+t3+t4) 0; sin(t1+t2+t3+t4) cos(t1+t2+t3+t4) 0; 0 1 1];
s5= [cos(t1+t2+t3+t4+t5) -sin(t1+t2+t3+t4+t5) 0; sin(t1+t2+t3+t4+t5) cos(t1+t2+t3+t4+t5) 0; 0 1 1];
s6= [cos(t1+t2+t3+t4+t5+t6) -sin(t1+t2+t3+t4+t5+t6) 0; sin(t1+t2+t3+t4+t5+t6) cos(t1+t2+t3+t4+t5+t6) 0; 0 1 1];
s7= [cos(t1+t2+t3+t4+t5+t6+t7) -sin(t1+t2+t3+t4+t5+t6+t7) 0; sin(t1+t2+t3+t4+t5+t6+t7) cos(t1+t2+t3+t4+t5+t6+t7) 0; 0 1 1];

start = T0_7;
desired1 = Txd;
desired2= Txd2;

L1= .55;
L2= .30;
L3= .18;

k1= [L1*cos(theta1+theta2) L2*cos(theta1+theta2+theta4) L3*cos(theta1+theta2+theta4+theta5)];
k2= [L1*sin(theta1+theta2) L2*sin(theta1+theta2+theta4) L3*sin(theta1+theta2+theta4+theta5)];
k3 = [L1*cos(theta3) L2*cos(theta3+theta4) L3*sin(theta3+theta4+theta7)];

theta = zeros(7, 1); 
max_iterations = 100;  

% Newton-Raphson method for inverse kinematics
for iter = 1:max_iterations






end

function x = compute_end_effector_position(theta)

    s1 = [cos(theta(1)), -sin(theta(1)), 0; sin(theta(1)), cos(theta(1)), 0; 0, 0, 1];
    s2 = [cos(theta(1) + theta(2)), -sin(theta(1) + theta(2)), 0; sin(theta(1) + theta(2)), cos(theta(1) + theta(2)), 0; 0, 1, 1];
    s3 = [cos(theta(1) + theta(2) + theta(3)), -sin(theta(1) + theta(2) + theta(3)), 0; sin(theta(1) + theta(2) + theta(3)), cos(theta(1) + theta(2) + theta(3)), 0; 0, 1, 1];
    s4 = [cos(theta(1) + theta(2) + theta(3) + theta(4)), -sin(theta(1) + theta(2) + theta(3) + theta(4)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4)), cos(theta(1) + theta(2) + theta(3) + theta(4)), 0; 0, 1, 1];
    s5 = [cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), -sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), 0; 0, 1, 1];
    s6 = [cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), -sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), 0; 0, 1, 1];
    s7 = [cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), -sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), 0; 0, 1, 1];


    x = s1 * s2 * s3 * s4 * s5 * s6 * s7 ;
end
