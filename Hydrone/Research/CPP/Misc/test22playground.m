%% Test 22 playground %%

clc
clear
close all



%% add survey file names

survey=readtable("TestData\Test_22\Test22_outlinelow.txt");

outline= readtable("TestData\Test_22\Test22_outlinehigh.txt");

base = readtable("TestData\Test_22\ncdot33801.txt");

%wind=readtable("TestData\Test_9\test9_wind.txt");

%% survey convert GPS to (m)

refLat= (base.Var4(4,1)+base.Var4(5,1))*.5;  % Latitude of Charlotte, NC
refLon = (base.Var3(4,1)+base.Var3(5,1))*.5; % Longitude of Charlotte, NC
refAlt = ((base.Var5(4,1)+1.8)+(base.Var5(5,1)+1.8))*.5;        % Altitude of Charlotte, NC 

lat0=survey.GPSLatitude;
long0=survey.GPSLongitude;
alt0=survey.Altitude;

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

% minlat=min(enu(:,1));
% minlon=min(enu(:,2));

survey.GPSLatitude=enu(:,1);
survey.GPSLongitude=enu(:,2);
survey.Altitude=enu(:,3);

%% outline convert GPS to (m)

lat0=outline.GPSLatitude;
long0=outline.GPSLongitude;
alt0=outline.Altitude;

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

outline.GPSLatitude=enu(:,1);
outline.GPSLongitude=enu(:,2);
outline.Altitude=enu(:,3);

%% path convert GPS to (m)
% 
% lat0=path.Var2;
% long0=path.Var3;
% alt0=zeros(1,size(path.Var3,1))';
% 
% enu = lla2enu([lat0, long0, alt0], [base.Var3(5,1), base.Var4(5,1), base.Var5(5,1)], 'ellipsoid');

waterline=base.Var5(5,1);


figure;
hold on

plot(survey.GPSLatitude,survey.Altitude)
yline(refAlt,'k')
yline(waterline,'b')
title('unadjusted alt')
legend('Survey Altitude','Wing Meas','Waterline')
xlabel('X (m)')
ylabel('depth (m)')
hold off



figure;
hold on
scatter(survey.GPSLatitude,waterline-survey.Depth_m-refAlt)
scatter(outline.GPSLatitude,waterline-outline.Depth_m-refAlt)
yline(refAlt-refAlt,'k')
yline(waterline-refAlt,'b')
title('Adjusted alt')
legend('Survey Depth Low','Survey Depth High','Wing Meas/zero','Waterline')
xlabel('X (m)')
ylabel('depth (m)')

hold off