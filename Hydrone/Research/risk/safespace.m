%% safe Space %%

clc
clear
close all
% 
syms s

%% HW 6 Problem 1










% A = [-8.43*10^-4 7.42*10^-4; .004 -.004];
% B=[-.055 -5*10^-5 0; 1.03 0 1];
% C=[1 0];
% D=[0 0 0];
% 
% [num1,den1]=ss2tf(A,B,C,D,1);
% 
% [num2,den2]=ss2tf(A,B,C,D,2);
% 
% [num3,den3]=ss2tf(A,B,C,D,3);
% 
% tf1 = tf(num1, den1);
% tf2 = tf(num2, den2);
% tf3 = tf(num3, den3);

% A=[0 1;38.5*10 0]
% eigs(A)



% 
% load("grav_sens.mat")

%% Hw 4 problem 1 %%
% avg=mean(x)
% std= (sum((x-avg).^2))/(size(x,1))^.5;
% y=pdf('normal',x,avg,std)
% hold on
% scatter(x,y)
% xlabel('x')
% ylabel('density')
% xlim([avg-4*std,avg+4*std])
% title('Hwproblem 1')


%% Hw 4 problem 3 %%

% P = [1 4;4 20];
% avg = [5;15];
% 
% [V,M]=eigs(P);
% phi= atan2(V(2,1),V(1,1));
% 
% 
% tag1=[13,20]';
% tag2=[10,23]';
% 
% e1=avg-tag1;
% e2=avg-tag2;
% 
% distance1=norm(e1);
% distance2=norm(e2);
% 
% c=(-2*log((1-.95)))^.5;
% 
% c1=(e1'*inv(P)*e1)^.5;
% c2=(e2'*inv(P)*e2)^.5;
% 
% 
% max=M(1,1);
% min=M(2,2);
% 
% 
% theta= linspace(0,2*pi,100);
% x=avg(1,:) + c*max^.5*cos(phi)*cos(theta)-c*min^.5*sin(phi)*sin(theta);
% y=avg(2,:) + c*max^.5*sin(phi)*cos(theta)+c*min^.5*cos(phi)*sin(theta);
% 
% x1=avg(1,:) + c1*max^.5*cos(phi)*cos(theta)-c1*min^.5*sin(phi)*sin(theta);
% y1=avg(2,:) + c1*max^.5*sin(phi)*cos(theta)+c1*min^.5*cos(phi)*sin(theta);
% 
% x2=avg(1,:) + c2*max^.5*cos(phi)*cos(theta)-c2*min^.5*sin(phi)*sin(theta);
% y2=avg(2,:) + c2*max^.5*sin(phi)*cos(theta)+c2*min^.5*cos(phi)*sin(theta);
% 
% hold on
% scatter(avg(1,1),avg(2,1),"r",Marker="+")
% plot(x,y,"r",linestyle="--")
% 
% scatter(tag1(1,1),tag1(2,1),"b",Marker="+")
% plot(x1,y1,"b",linestyle="--")
% 
% scatter(tag2(1,1),tag2(2,1),"g",Marker="+")
% plot(x2,y2,"g",linestyle="--")
% 
% legend('95% ellipse','mean','tag1 ellipse','tag1 measurment','tag2 ellipse','tag2 measurment')
% xlabel('x1')
% ylabel('x2')
% hold off



%% hw 4 problem 4 %%
% rng(3)
% A=[1 .25;-.5 .1] ;
% B=[0;1];
% C=[-1 1];
% 
% D=0;
% 
% sys = ss(A,B,C,D);
% 
% x0=[2;-1];
% 
% w=.01;
% b=1.7;
% 
% Q=eye(2)*.001;
% h=1;
% T=100;
% 
% R=.5;
% 
% muw = zeros(2,1);
% muv = 0;
% 
% % sim
% N=T/h;
% thist(1)=0;
% xhist(:,1)=x0;
% yhist(1)=C*x0;
% uhist(1)=sin(0);
% for k=2:1:N
% 
% xkm1=xhist(:,k-1);
% tkm1= thist(k-1);
% ukm1 = sin(w*k^b);
% 
% % % %%  uncomment for no noise
% % % wkm1=mvnrnd(muw,Q*k);
% % % xhist(:,k)=A*xkm1 + B*ukm1 %+wkm1';
% % % thist(k) = thist(k-1) + h;
% % % %vk = mvnrnd(muv, R/h);
% % % yhist(k) = C*xhist(:,k)%+vk;
% % % uhist(k) = ukm1;
% 
% %% uncomment for noise
% wkm1=mvnrnd(muw,Q*k);
% xhist(:,k)=A*xkm1 + B*ukm1 +wkm1';
% thist(k) = thist(k-1) + h;
% vk = mvnrnd(muv, R/h);
% yhist(k) = C*xhist(:,k)+vk;
% uhist(k) = ukm1;
% 
% 
% end
% 
% 
% subplot(1,3,1)
% plot(thist,uhist(1,:))
% xlabel('time (s)')
% ylabel('input (u)')
% legend('u')
% 
% 
% 
% subplot(1,3,2)
% hold on
% plot(thist,xhist(1,:))
% plot(thist,xhist(2,:))
% xlabel('time (s)')
% ylabel('state (x)')
% legend ('x1','x2')
% hold off
% 
% 
% subplot(1,3,3)
% plot(thist,yhist(1,:))
% legend('y')
% xlabel('time (s)')
% ylabel('output (y)')
