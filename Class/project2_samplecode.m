clc
clear
close all

% Read the pickle file using Pytho
%py=pyenv('Version','C:\Users\alexn\AppData\Local\Programs\Python\Python39\python.exe');
fid=py.open('C:\Users\alexn\Documents\GitHub\anikonow\Class\data\pt1_data.pkl','rb');
data=py.pickle.load(fid);
data=struct(data);


%%% Unpack the ground truth data

gtp=double(data.gt.p);   % ground truth position in inertial frame
gtv=double(data.gt.v);   % ground truth velocity in inertial frame
gtr=double(data.gt.r);   % ground truth heading in inertial frame
gtw=double(data.gt.w);   % ground truth rate of heading in inertial frame
gta=double(data.gt.a);   %% ground truth rotational acceleration of the vehicle,
gtalpha=double(data.gt.alpha);
deltax(1,:)=[gtp(1,:) gtv(1,:) gtr(1,:)];   %%% ground truth of 9 ate

% Let's plot the ground truth trajectory to see what it looks like. When you're testing your
% code later, feel free to comment this out.
%
gt_fig = figure;
% Create a 3D plot
ax = axes('Parent', gt_fig, 'Projection', 'orthographic');
% Plot the data
plot3(ax, gtp(:,1), gtp(:,2), gtp(:,3));
hold on
% Label the axes (optional)
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
% Add title (optional)
title('3D Plot of ground truth trajectory');
legend

%%%% Unpack the Data from sensors
lidar=double(data.lidar.data);     %%% Lidar Data
lidar_t=double(data.lidar.t)';     %%%% Time vector of Lidar
gnss=double(data.gnss.data);       %%%% GNSS sensor data
gnss_t=double(data.gnss.t)';        %%% Time vector of GNSS
imu_w=double(data.imu_w.data);     %%%% IMU data for rotational Speed
imu_f=double(data.imu_f.data);     %%%% IMU data for proper accel.
imu_f_t=double(data.imu_f.t)';     %%%% Time vectro for IMU data
imu_f_t=imu_f_t(1:length(imu_f),1);

% Correct calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.1).
C_li_correct = double([ 0.99376, -0.09722,  0.05466;
    0.09971,  0.99401, -0.04475;
    -0.04998,  0.04992,  0.9975]);

% Incorrect calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.05).
% C_li_incorrect = [
%      0.9975 , -0.04742,  0.05235;
%      0.04992,  0.99763, -0.04742;
%     -0.04998,  0.04992,  0.9975 ];

t_i_li =( [0.5, 0.1, 0.5]);   %%% translation

%Remember that our LIDAR data is actually just a set of positions estimated from a separate
% scan-matching system, so we can insert it into our solver as another position measurement,
% just as we do for GNSS. However, the LIDAR frame is not the same as the frame shared by the
% IMU and the GNSS. To remedy this, we transform the LIDAR data to the IMU frame using our
% known extrinsic calibration rotation matrix C_li and translation vector t_i_li.

% Transform from the LIDAR frame to the vehicle (IMU) frame.
lidardata = (C_li_correct * lidar')' + (t_i_li);

plot3(ax, lidardata(:,1), lidardata(:,2),lidardata(:,3),'r')
hold on
plot3(ax, gnss(:,1), gnss(:,2), gnss(:,3),'g');


%% Constants
% Now that our data is set up, we can start getting things ready for our solver.
% One of the most important aspects of a filter is setting the estimated sensor variances correctly.
% We set the values here.

var_imu_f = 0.10;
var_imu_w = 0.25;
var_gnss  = 0.01;
var_lidar = 1;

% We can also set up some constants that won't change for any iteration of our solver.

g = [0, 0, -9.81];  % gravity
l_jac = zeros(9, 6);
l_jac(4:end, :) = eye(6);  % motion model noise jacobian
L=l_jac;
h_jac = zeros(3, 9);
h_jac(:, 1:3) = eye(3);  % measurement model jacobian
H=h_jac;

%% Initial Values
% Let's set up some initial values for our ES-EKF solver.

p_check = zeros(size(double(data.imu_f.data), 1), 3);  % position estimates
v_check = zeros(size(double(data.imu_f.data), 1), 3);  % velocity estimates
q_check = zeros(size(double(data.imu_f.data), 1), 4);  % orientation estimates as quaternions
p_cov_check= zeros( 9, 9,size(double(data.imu_f.data), 1));  % covariance matrices at each timestep

% Set initial values.
p_check = gtp(1, :);
v_check = gtv(1, :);
q_check = eul2quat(gtr(1, :),'XYZ');
p_cov_check = zeros(9,9,2);  % covariance of estimate
gnss_i  = 0;
lidar_i = 0;

x=0;
y=0;
z=0;
% 5. Main Filter Loop
for k = 2:length(imu_f_t)%
    % size(imu_f)  % start at 2 because we have initial prediction from gt
    dt = imu_f_t(k) - imu_f_t(k - 1);
    %Cns(:,:,k-1)=quat2rotm(q_check(k-1, :));

    % 1. Update state with IMU inputs
    x= x+ v_check(1,1)* cos(imu_f(k-1,3))*dt
    y= y+ v_check(1,1)* sin(imu_f(k-1,3))*dt
    
    % 1.1 Linearize the motion model and compute Jacobians

    % 2. Propagate uncertainty
    
    % 3. Check availability of GNSS and LIDAR measurements


%% 

    if ismember(imu_f_t(k),lidar_t)
        [index_lidar,col_lidar,tlidar]=find(lidar_t==imu_f_t(k))
        y(k,:)=lidardata(index_lidar,:);
        Rlidar=var_lidar*eye(3);
        disp('lidar')
        [p_hat, v_hat, q_hat, p_cov_hat] = ef_measurement_update(Rlidar, p_cov_check(:,:,k), y(k,:), p_check(k-1,:), v_check(k-1,:), q_check(k-1,:))


        % Update states (save)


    elseif ismember(imu_f_t(k),gnss_t);
        [index_gnss,col_gnss,tgnss]=find(gnss_t==imu_f_t(k));
        y(k,:)=gnss(index_gnss,:);
        Rgnss=var_gnss*eye(3);
        disp('gnss')

        [p_hat, v_hat, q_hat, p_cov_hat] = ef_measurement_update(Rgnss, p_cov_check(:,:,k-1), y(k,:), p_check(k-1,:), v_check(k-1,:), q_check(k-1,:))


        % Update states (save)

    end

fprintf('loops ran %f\n', k)    
end


figure

subplot(221)
plot(imu_f_t,p_check(:,1))
hold on
% plot(gtp(:,1),'r--')
hold on
plot(gnss_t,gnss(:,1),'g-.')
hold on
plot(lidar_t,lidardata(:,1),'k')

subplot(222)
plot(imu_f_t,p_check(:,2))
hold on
% plot(gtp(:,2),'r--')
hold on
plot(gnss_t,gnss(:,2),'g-.')
hold on
plot(lidar_t,lidardata(:,2),'k')

subplot(223)
plot(imu_f_t,p_check(:,3))
hold on
% plot(gtp(:,3),'r--')
hold on
plot(gnss_t,gnss(:,3),'g-.')
hold on
plot(lidar_t,lidardata(:,3),'k')

wdfwdfwdfw

figure
subplot(221)
plot(imu_f_t,v_check(:,1))
hold on
plot(gtv(:,1),'r--')

subplot(222)
plot(imu_f_t,v_check(:,2))
hold on
plot(gtv(:,2),'r--')

subplot(223)
plot(imu_f_t,v_check(:,3))
hold on
plot(gtv(:,3),'r--')

legend('vcheck','true')


figure
plot3(gtp(:,1),gtp(:,2),gtp(:,3),'r--');
hold on
plot3(p_check(:,1),p_check(:,2),p_check(:,3),'b');

function [p_hat, v_hat, q_hat, p_cov_hat] = ef_measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check)

deltax=zeros(9,1)
H=eye(3,9);

p_hat = p_check;
v_hat = v_check;
q_hat = q_check;
p_cov_hat = p_cov_check;

R = diag(sensor_var);

% Compute Kalman Gain
K = p_cov_hat * H' / (H * p_cov_hat * H' + R);
% Compute error state
deltax = K * (y_k-H* [p_hat ; v_hat ; quat2eul(q_hat)]);
% Correct predicted state

% Compute corrected covariance
p_cov_hat= (eye(9)-K*H)*p_cov_check;
%  return p_hat, v_hat, q_hat, p_cov_hat
p_hat=p_hat+deltax(1:3);
v_hat=v_hat+deltax(4:6);
q_hat=quatmultiply(q_hat, eul2quat(deltax(7:9)', 'XYZ'));

end



function M = crossMat(v)
M = [0, -v(3), v(2);
    v(3), 0, -v(1);
    -v(2), v(1), 0];
end
