
%% intelroboticshw4 %%


clc
clear

syms L d t theta1 theta2 theta3 theta4 theta5 theta6 theta7;



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

% Newton-Raphson 
for iter = 1:max_iterations
% 
% 
% function xd-f(theta)
%
% delta (theta) = f'(theta)^-1 (xd-f(theta0)
% 
% 

end

function x = pos(theta)

    s1 = [cos(theta(1)), -sin(theta(1)), 0; sin(theta(1)), cos(theta(1)), 0; 0, 0, 1];
    s2 = [cos(theta(1) + theta(2)), -sin(theta(1) + theta(2)), 0; sin(theta(1) + theta(2)), cos(theta(1) + theta(2)), 0; 0, 1, 1];
    s3 = [cos(theta(1) + theta(2) + theta(3)), -sin(theta(1) + theta(2) + theta(3)), 0; sin(theta(1) + theta(2) + theta(3)), cos(theta(1) + theta(2) + theta(3)), 0; 0, 1, 1];
    s4 = [cos(theta(1) + theta(2) + theta(3) + theta(4)), -sin(theta(1) + theta(2) + theta(3) + theta(4)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4)), cos(theta(1) + theta(2) + theta(3) + theta(4)), 0; 0, 1, 1];
    s5 = [cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), -sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5)), 0; 0, 1, 1];
    s6 = [cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), -sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6)), 0; 0, 1, 1];
    s7 = [cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), -sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), 0; sin(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), cos(theta(1) + theta(2) + theta(3) + theta(4) + theta(5) + theta(6) + theta(7)), 0; 0, 1, 1];


    x = s1 * s2 * s3 * s4 * s5 * s6 * s7 ;
end
