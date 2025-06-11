%% Depth Corrections %%

clear
clc

NED = zeros(3,1);
%all units in INCHES (inches) ((in)) at the moment
data = readtable('SensorTest_data.csv');
%% ASV IC and inputs
roll = data.roll;
roll= deg2rad(roll);
pitch = data.pitch;
pitch=deg2rad(pitch);
yaw = data.yaw;
yaw=deg2rad(yaw);
depth_sensor_reading = data.Depth;
latitude = data.Lat;
longitude =data.Lon;

tick = 10; % # of notches on water level gauge
water_level = [0; 0; 1.4 + (0.25 * tick)] .* (1/12);

% Sensor position vectors (in)
imu_vector = [0.0; 0.0; 0.0];  % IMU vector in the Vehicle frame (assumtion its 0,0,0)
ping_vector = [13.9; -5.2; 9.7];  % Ping vector in the Vehicle frame
gps_vector = [-4.5 ; -3 ; -8.8];%gps
water_vector = ping_vector - water_level;

%gps_corrections -> 1 inch = lat/lon decimal value (get somthing more better later)
Northing = .000000254; %lat/in
Easting = .000000254; %lon/in

% Angles in radians valid from -pi/4 to pi/4
for i = 1:size(data,1)

% roll = 0;
% pitch = 0;
% yaw = 0;
% depth_sensor_reading = 60; % Depth sensor reading (in) actually recorded in feet

%% Rotation Matrix
% Rotation matrix from Inertial to Vehicle
R_yaw = [cos(yaw(i)), -sin(yaw(i)), 0;
         sin(yaw(i)), cos(yaw(i)), 0;
         0, 0, 1];
R_pitch = [cos(pitch(i)), 0, sin(pitch(i));
           0, 1, 0;
           -sin(pitch(i)), 0, cos(pitch(i))];
R_roll = [1, 0, 0;
          0, cos(roll(i)), -sin(roll(i));
          0, sin(roll(i)), cos(roll(i))];

% Combine transformations
Fi_v = R_yaw * R_pitch * R_roll;

% raw depth vector
depth = [0; 0; depth_sensor_reading(i)];

%sensor position in vehical frame
sensor_position = Fi_v * (ping_vector);
water_postition = sensor_position + water_level;

%Depth measurement vertical correction
depthi_1 = R_yaw * depth;
depth_vert= depthi_1 .* cos(pitch(i)) .* cos(roll(i));
NED(3) = depth_vert(3) + water_level(3); %corrected depth

%raw collected depth corrected for NE calcs
NEdepth = Fi_v*depth;
gps_postition = Fi_v*gps_vector;

%corrected north and east vectors
NED(1)= depth(1)+(ping_vector(1) - gps_postition(1));
NED(2) = depth(2)+(ping_vector(2) - gps_postition(2));

%working on better corrections
GPS_Corrections_N = Northing * NED(1);
GPS_Corrections_E = Easting * NED(2);
Lat = latitude(i)+ GPS_Corrections_N;
Lon = longitude(i)+ GPS_Corrections_E;
Depth = NED(3);
corrected_data(i, :) = [Lat,Lon, Depth];
end
writematrix(corrected_data,'correctedsensortest.csv')

%Corrected depth downward
%corrected_depth = depth_vert(3) + (sensor_position(3) - water_level(3));


% fprintf('Uncorrected Depth: %.2f inches\n', depth_sensor_reading);
% fprintf('Corrected Depth: %.2f inches\n', NED(3));