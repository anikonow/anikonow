% Kriging plotting
% ASV Bathymetry Contour Plot
clc
clear
close all

% % Read data from SensorTest_data.csv
% data = readtable('TestData\Test_9\test9_survey_1.txt');%correctedsensortest.csv SensorTest_data.csv
% 
% %% Observations %%
% % Latitude, longitude, and depth data from csv
% long = data.GPSLongitude;  % Make sure column names match data names in csv.
% lat = data.GPSLatitude;
% depthData = data.Corr_Depth2;  % Correct the variable name.

%% Grid setup
% Resolution = 50;
% x = (max(long) - min(long)) / Resolution;
% y = (max(lat) - min(lat)) / Resolution;
% [X, Y] = meshgrid(min(long):x:max(long), min(lat):y:max(lat));
% interpolationPoints = [X(:), Y(:)];

Resolution = 50;
x = 0:.5:50;
y = x;
[X, Y] = meshgrid(x);
F= 6 * sin(0.1*X).*cos(0.1*Y)-2*cos(0.055*X-0.1*Y)-4.6;

% surf(X,Y,F,'EdgeColor','None');
% %[x_path, y_path] = ginput
% % ROI=[x_path,y_path]

load("Outlinesurvey.mat")

hold on
surf(X,Y,F,'EdgeColor','None');
%plot(ROI(:,1),ROI(:,2),'color','k','linewidth',2)

colorbar;

xlabel('x-axis (m)');
ylabel('y-axis (m)');
ylabel(colorbar,'z-axis (m)')
title('Actual Bathymetry');

hold off

XX = ROI(:,1);
YY = ROI(:,2);

F2= 6 * sin(0.1*XX).*cos(0.1*YY)-2*cos(0.055*XX-0.1*YY)-4.6;

% figure;
% plot3(XX,YY,F2)

interpolationPoints = [X(:), Y(:)];
data=F2;

Xq = [XX,YY];

gprMdl = fitrgp(Xq,data,'KernelFunction','squaredexponential');
[z,zcov] = predict(gprMdl,interpolationPoints);

z=reshape(z,size(X));
zcov=reshape(zcov,size(X));

figure;

hold on;
surf(X, Y, z, 'EdgeColor', 'none');  

%plot(ROI(:,1),ROI(:,2),'color','k','linewidth',2)
colorbar;
xlabel('x-axis (m)');
ylabel('y-axis (m)');
ylabel(colorbar,'z-axis (m)')
title('Kriged Data Set');
hold off


figure;

hold on;
%surf(X, Y, z, 'EdgeColor', 'none');  
contourf(X, Y, z,[-2.5,-20]);
%plot(ROI(:,1),ROI(:,2),'color','k','linewidth',2)
colorbar;
xlabel('x-axis (m)');
ylabel('y-axis (m)');
ylabel(colorbar,'z-axis (m)')
title('Contour Data Set');
hold off


% figure;
% surf(X, Y, z + zsd, 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
% hold on;
% surf(X, Y, z - zsd, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
% plot3(long, lat, depthData, 'ro', 'MarkerFaceColor', 'r');
% xlabel('Longitude');
% ylabel('Latitude');
% zlabel('Depth');
% title('2D Kriging with 1 Std Deviation Bounds');
% colorbar;
% grid on;




figure;
hold on
surf(X, Y, zcov, 'EdgeColor', 'none');  
plot(ROI(:,1),ROI(:,2),'color','k','linewidth',2)
xlabel('x-axis (m)');
ylabel('y-axis (m)');
ylabel(colorbar,'Variance')
title('Kriged Variance');
colormap(flipud(gray))
caxis([0 .05])

