clc
clear
close all

%% fid=py.open([path to file],'rb')
%py=pyenv('Version','C:\Users\alexn\AppData\Local\Programs\Python\Python39\python.exe');
%pyenv('Version', '3.9')
fid=py.open(['data.pickle'],'rb');  %%% [the path where the data is located]


data=py.pickle.load(fid);
data=struct(data);

t=double(data.t);   %# timestamps [s]
x_init=double(data.x_init);   % initial x position [m]
y_init=double(data.y_init);   % initial y position [m]
th_init=double(data.th_init);    %initial theta position [rad]
v=double(data.v);   % translational velocity input [m/s]
v_var=double(data.v_var);
om=double(data.om);   %rotational velocity input [rad/s]
om_var=double(data.om_var);

% bearing and range measurements, LIDAR constants
l=double(data.l);  %x,y positions of landmarks [m]
xl=l(:,1);
yl=l(:,1);
d=double(data.d)   %distance between robot center and laser rangefinder [m]
b=double(data.b);   % bearing to each landmarks center in the frame attached to the laser [rad]
b_var=double(data.b_var);
r=double(data.r);    % range measurements [m]
r_var=double(data.r_var);




%%%% Initialization
v_var = 0.01;  % translation velocity variance
om_var =.01;  % rotational velocity variance
r_var = .1;  % range measurements variance
b_var = 0.1;  % bearing measurement variance

Qk = diag([v_var, om_var]); % input noise covariance
R_y =diag([r_var, b_var]);  % measurement noise covariance

x_est = zeros(3,[length(v)]); % estimated states, x, y, and theta
P_est = zeros(3,3,[length(v)]);  % state covariance matrices

x_est(:,1) = [x_init, y_init, th_init]'; %# initial state
P_est(:,:,1) = diag([1, 1, 0.1]); % initial state covariance


%%% main Loop
xcheck= zeros(1,3)

for k=2:length(t);
    dt=t(k)-t(k-1);
    x_est(3,k-1) = wrapToPi(x_est(3,k-1));
    %%%% Calcualte xcheck
    
    xcheck(1,1) = x_est(k-1,1)+v(k-1)*cos(xcheck(1,3))*dt;
    xcheck(1,2) = x_est(k-1,2)+v(k-1)*sin(xcheck(1,3))*dt;
    xcheck(1,3) = x_est(k-1,3) + tan(xcheck(1,2)/xcheck(1,1))*dt;
    %%% Caclulate Fk and Lk and Pcheck
    Fk = xcheck;
    Lk = eye(3); 
    P_check = Fk * P_est(:, :, k-1) * Fk' + Lk * Qk * Lk';

    



    %%%% The loop below updates xcheck and Pcheck using the infomration
    %%%% from Lidar sensors about the 8 landmarks
    for i=1:length(r(k,:))
        [ x_check, P_check] = measurement_update(l(i,:), r(k, i), b(k, i),...
                                                 P_check, x_check,Qk, R_y);
    end

    %Set final state predictions for timestep
    x_est(1, k) = x_check(1);
    x_est(2, k) = x_check(2);
    x_est(3, k) = x_check(3);
    P_est(:, :, k) = P_check;

end

subplot(121)
plot(x_est(1,:),x_est(2,:));
subplot(121)
plot(t,x_est(3,:));




function [x_check,P_check]=measurement_update(lk, rk, bk, P_check, x_check,Q, Rk)

%1. Compute measurement Jacobian

Hk=zeros(2,3);
bk=wrapToPi(bk);
ym=[rk;bk];


syms xl xk thk  d yl yk
dx=xl-xk-d*cos(thk);
dy=yl-yk-d*sin(thk);
r=sqrt(dx^2+dy^2);
phi=atan2(dy,dx)-thk;

dr=jacobian(r,[xk, yk, thk]);
dphi=jacobian(phi,[xk, yk, thk]);

d=0;
xl=lk(1);
yl=lk(2);
xk=x_check(1,:);
yk=x_check(2,:);
thk=x_check(3,:);


Hk=[eval(dr);eval(dphi)];
Mk=eye(2);

phi = wrapToPi(eval(phi));
y_check=eval([r;phi]);

% 2. Compute Kalman


Kk=P_check*Hk'*inv(Hk*P_check*Hk'+Mk*Rk*Mk');

% 3. Correct predicted state (remember to wrap the angles to [-pi,pi])

x_hat=x_check+Kk*(ym-y_check);


% 4. Correct covariance

P_hat=(eye(3)-Kk*Hk)*P_check;

x_check=x_hat;
P_check=P_hat;

end


function x = wraptopi(x)
if x > pi
    x = x - (floor(x / (2 * pi)) + 1) * 2 * pi;
elseif x < -pi
    x = x + (floor(x / (-2 * pi)) + 1) * 2 * pi;
end
end


