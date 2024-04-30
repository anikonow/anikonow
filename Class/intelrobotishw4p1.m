%% hw 4 intel robotics P1 %%

clc
clear

x_desired = 3;  
y_desired = 2;  
theta_desired = 0;  



L1 = 3;  
L2 = 2;  
L3 = 1;
    
xb=x_desired;
yb=y_desired;


beta= acos((xb^2+yb^2-L1^2-L2^2)/(2*L1*L2));
gamma= atan2(yb,xb);
alpha= atan2((L2*sin(pi-beta)),L1+L2*cos(pi-beta));
    
%beta=.5879;

theta1=gamma-alpha;
theta1_2 = alpha+gamma;
    
theta2=pi-beta;
theta2_2= beta-pi;

theta3= theta1+theta2-pi;
theta3_2= pi-theta1_2+theta2_2;

x1_1=L1*cos(theta1) +L2*cos(theta1+theta2) + L3*cos(theta3 +theta1 + theta2);
y1_1=L1*sin(theta1) +L2*sin(theta1+theta2) +L3*sin(theta3 +theta1 + theta2);

x1_2=L1*cos(theta1_2) +L2*cos(theta1_2+theta2_2) + L3*cos(theta3_2 +theta1_2 + theta2_2);
y1_2=L1*sin(theta1_2) +L2*sin(theta1_2+theta2_2) +L3*sin(theta3_2 +theta1_2 + theta2_2);


disp('first solution')
disp(x1_1)
disp(y1_1)
fprintf('%f %f %f\n',theta1, theta2, theta3)
disp('second solution')
disp(x1_2)
disp(y1_2)
fprintf('%f %f %f\n',theta1_2, theta2_2, theta3_2)
