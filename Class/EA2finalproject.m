%% EA2 Project %%

clc
clear
close all


syms m1 m2 c1 c2 k1 k2 lamda t alpha beta


alpha= 1;
beta= 1;


k1=6; %N/m
k2=4;
c1=2; %(N-s)/m
c2=1;
m1=2; %Kg
m2=1;

K = [k1+k2 -k2; -k2 k2];
%C= [c1+c2 -c2; -c2 c2];

M = [m1 0; 0 m2];
C=alpha*M+beta*K;


[V,D]= eig(K,M);


%V = eigen vectors
%D = eigen values

x0= [1;0];%initial x cord and velocity
x02=[0;0]; %initial x cord and velo

%xdot0= [0;0];
tspan=[0,10];

%P=[-1 1; 1.186 1.686];
%P=[V];

x0=V^-1*x0; %solve for inital model cord
x02=V^-1*x02;


M_d= V'*M*V;
C_d= V'*C*V;
%C_d=[-2.551 0; 0 -5.430];
K_d= V'*K*V;

%F1=@(t) -2.272*cos(3*t); %N
C*M^-1*K
K*M^-1*C


[t, x] = ode45(@ode1, tspan, x0);
[t2, y] = ode45(@ode2, tspan, x02);

% Extract the solution
x=V*x';
y=V*y';

x1 = x(1, :);  % x(t)
x2 = x(2, :);  % dx/dt(t)

y1 = y(1,:);
y2 = y(2,:);

% Plot the results
figure;
subplot(2, 1, 1);
plot(t, x1, 'b', 'LineWidth', 1.5);
hold;
plot(t2, y1, 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('Distance');
title('Displacement vs time');

subplot(2, 1, 2);
plot(t, x2, 'b', 'LineWidth', 1.5);
hold;
plot(t2, y2, 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('Velo');
title('Velocity vs time');
hold off

function dxdt = ode1(t, x)
    x1 = x(1);  % x1(t)
    x2 = x(2);  % x2(t)

    c = 2.62;
    k = 1.627;
    F = 2.272;
    
    dx1dt = x2;                    % dx1/dt = x2
    dx2dt = -c*x2 - k*x1 - F*cos(3*t);  % dx2/dt = -c1*x2 - k1*x1 - P'*5cos(3*t)
    
    
    dxdt = [dx1dt; dx2dt];
end

function dxdt = ode2(t, x)
    
    x1 = x(1);  % x1(t) = x(t)
    x2 = x(2);  % x2(t) = dx/dt

    c = 8.37;
    k = 7.372;
    F = 3.831;
    
    dx1dt = x2;                    % dx1/dt = x2
    dx2dt = -c*x2 - k*x1 - F*cos(3*t);  % dx2/dt = -c2*x2 - k2*x1 - P'*5*cos(3*t)
    dxdt = [dx1dt; dx2dt];
end




