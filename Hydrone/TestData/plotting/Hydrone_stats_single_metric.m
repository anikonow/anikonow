%% test code prep %%

clear
clc
close all

%% To-Do
%fix new corrected data legend/reformat depth data
%add a function that cleans errors in outline+survey data


% retroactively change raw data to corrected data
% Fixed, corr and uncorrected data now is collected

% get waypoints in UTM 17 or GPS coords
% fixed (export .L84 waypoint file)


%% add survey file names

survey=readtable("TestData\Test_21\Test21_survey.txt");

outline=readtable("TestData\Test_17\Test17_outline.txt");

path=readtable("TestData\Test_17\Test17_path.txt");

%wind=readtable("TestData\Test_9\test9_wind.txt");

%% convert GPS to (m)

% % refLat= 35.2271;  % Latitude of Charlotte, NC
% % refLon = -80.8431; % Longitude of Charlotte, NC
% % refAlt = 0;        % Altitude of Charlotte, NC 
% % 
% % lat0=survey.GPSLatitude;
% % long0=survey.GPSLongitude;
% % alt0=zeros(1,size(survey.GPSLatitude,1))';
% % 
% % enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');
% % 
% % minlat=min(enu(:,1));
% % minlon=min(enu(:,2));
% % 
% % survey.GPSLatitude=enu(:,1)-minlat;
% % survey.GPSLongitude=enu(:,2)-minlon;


%% DOL
distanceoffline = survey.DOL;
maxdis=max(abs(distanceoffline));
%buffer for max DOL calculation
if maxdis > 5 
    maxdis=5;
end

for p=1:length(survey.DOL)
    if survey.DOL(p)>maxdis || survey.DOL(p)<-maxdis
       survey.DOL(p) = 0;
    end
end


% % figure;
% % hold on
% % 
% % 
% % % Define rotation angle in radians
% % theta = deg2rad(320);
% % 
% % % Convert lat/lon into Cartesian coordinates (assuming small distances)
% % % Here, treat lat/lon as x/y for local rotation
% % x = survey.GPSLongitude; % Longitude as x-coordinate
% % y = survey.GPSLatitude; % Latitude as y-coordinate
% % 
% % % Apply rotation matrix
% % x_rot = cos(theta) * x - sin(theta) * y;
% % y_rot = sin(theta) * x + cos(theta) * y;
% % 
% % % Rotated latitude and longitude (interpreted as Cartesian rotation)
% % lon_rot = x_rot;
% % lat_rot = y_rot;
% % 
% % % Display results
% % %plot(lon_rot,lat_rot)
% % 
% % 
% % survey.NumSats= abs(survey.NumSats);
% % z_norm = ((survey.NumSats) - min(survey.NumSats)) / ((max(abs(survey.NumSats))- min(survey.NumSats)));
% % cmap=jet(256);
% % figure;
% % hold on
% % 
% % normsats= survey.NumSats/max(survey.NumSats);
% % scatter(lon_rot,normsats,'b','MarkerFaceAlpha',.1)
% % scatter(lon_rot,survey.Dop,'r')
% % 
% % psats = polyfit(lon_rot,normsats,16);
% % pdop = polyfit(lon_rot,survey.Dop,16);
% % 
% % lonfit = linspace(25,65,1000);
% % 
% % fitsats=polyval(psats,lonfit);
% % fitdop=polyval(pdop,lonfit);
% % 
% % plot(lonfit,fitsats,'b--', 'LineWidth', 1.5)
% % plot(lonfit,fitdop,'r--', 'LineWidth', 1.5)
% % 
% % 
% % xlim([25,65])
% % xline(40)
% % xline(50)
% % 
% % 
% % hold off
% % 
% % 
% % title('GNSS quality')
% % xlabel('Position (m)')
% % ylabel('Value')
% % 
% % 
% % legend('Sat % Data','DOP','Sat % LoBF','DOP LoBF','Bridge Pos')

survey.DOL= abs(survey.DOL);
z_norm = (survey.DOL - min(survey.DOL)) / ((max(abs(survey.DOL))));
z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);
figure;
for k=1:length(survey.GPSLongitude)-1
    plot(survey.GPSLongitude(k:k+1),survey.GPSLatitude(k:k+1),'Color',z_color(k,:),"linewidth",3)
    %fprintf('loop ran %f\n',k)
end
scatter(path.Var3,path.Var2,'r',"linewidth",1,"Marker","+")
colorbar;
colormap(jet)
plot(outline.GPSLongitude,outline.GPSLatitude,"k")
clim([min(survey.DOL) 3])
xlabel("Longitude (DD)")
ylabel("Latitude (DD)")
xlabel(colorbar, 'Distance (m)');
title("Waypoint Deviation")
axis equal


figure;
hold on
subplot(2,1,1)
plot(survey.Time,survey.DOL,"k")
yline(0,"r-")
xlabel("Time")
ylabel("Distance (m)")
title("Distance off Line")
subplot(2,1,2)
plot(survey.Time,survey.DBL,"k")
title("Distance from Begining Line")
xlabel("Time")
ylabel("Distance (m)")
sgtitle("Distance From Waypoints")
hold off


%% Roll Pitch Yaw

figure;
hold on
plot(survey.Time, survey.Pitch)
plot(survey.Time, survey.Roll)
plot(survey.Time, survey.COG-180)
legend("Pitch","Roll","Yaw")
title("IMU measurments")
xlabel("Time")
ylabel("Angle Degrees")
hold off


%% Speed Plot

surveyspeed= 2; %ft/s    .5m/s
avgss=mean(survey.Speed);
avgouts=mean(outline.Speed);

figure;
hold on
subplot(2,1,1);
plot(survey.Time,survey.Speed,"Color","k");
yline(surveyspeed,"r-")
yline(avgss,"b-")
ylabel("Speed (kn)")
xlabel("Time")
legend ("ASV Survey Speed","Set Survey Speed","Avg Survey Speed")
subplot(2,1,2)
plot(outline.Time,outline.Speed,"Color","k")
yline(avgouts,"b-")
ylabel("Speed (kn)")
xlabel("Time")
legend("RCV Survey Speed","Avg Survey Speed")
sgtitle("USV Survey Speed")




%% Depth
rawerror= abs(survey.RawDepth1-survey.RawDepth2);
correrror= abs(survey.Corr_Depth1-survey.Corr_Depth2);
lowerror= abs(survey.RawDepth1-survey.Corr_Depth1);
higherror= abs(survey.RawDepth2-survey.Corr_Depth2);


figure;
hold on 
plot(survey.Time, survey.Corr_Depth1,"color","b","LineWidth",2)
plot(survey.Time, survey.Corr_Depth2,"color","r","LineWidth",2)
plot(survey.Time, survey.RawDepth1,"color","[0,.5,1]","LineStyle","--")
plot(survey.Time, survey.RawDepth2,"color","[1,.2,0]","LineStyle","--")
plot(survey.Time, rawerror,"color","k")
%hold off
%plot(survey.Time, correrror)
legend("High Freq Corr","Low Freq Corr","High Freq Raw","Low Freq Raw", "Difference")
title("Corrected Dual Frequency Echosounder")
xlabel("Time")
ylabel("Depth (m)")

hold off


survey.Corr_Depth2= abs(survey.Corr_Depth2);
z_norm = ((survey.Corr_Depth2) - min(survey.Corr_Depth2)) / ((max(abs(survey.Corr_Depth2))- min(survey.Corr_Depth2)));
cmap=jet(256);

%z_norm=ones(363,1); %only uncomment if #of sats is same thru test

z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);

outline.Corr_Depth2= abs(outline.Corr_Depth2);
z1_norm = ((outline.Corr_Depth2) - min(outline.Corr_Depth2)) / ((max(abs(outline.Corr_Depth2))- min(outline.Corr_Depth2)));

z1_color = interp1(linspace(0,1,size(jet,1)),jet,z1_norm);

figure;
hold on
for k=1:length(survey.GPSLongitude)-1
    plot(survey.GPSLongitude(k:k+1),survey.GPSLatitude(k:k+1),'Color',z_color(k,:),"linewidth",3)
    %fprintf('loop ran %f\n',k)
end

for k=1:length(outline.GPSLongitude)-1
    plot(outline.GPSLongitude(k:k+1),outline.GPSLatitude(k:k+1),'Color',z1_color(k,:),"linewidth",3)
    %fprintf('loop ran %f\n',k)
end
scatter(path.Var3,path.Var2,'r',"linewidth",1,"Marker","+")
colorbar;
colormap(jet)
clim([min(survey.Corr_Depth2) max(survey.Corr_Depth2)])
%caxis([0 max(survey.NumSats)]) only uncommmetn if sat# is the same


title("Low Frequency Depth")
xlabel("Longitude (DD)")
ylabel("Latitude (DD)")
ylabel(colorbar,"Depth (m)")
axis equal
hold off










%% add plot for gps health


survey.NumSats= abs(survey.NumSats);
z_norm = ((survey.NumSats) - min(survey.NumSats)) / ((max(abs(survey.NumSats))- min(survey.NumSats)));
cmap=jet(256);

%z_norm=ones(363,1); %only uncomment if #of sats is same thru test

z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);


figure;
hold on
for k=1:length(survey.GPSLongitude)-1
    plot(survey.GPSLongitude(k:k+1),survey.GPSLatitude(k:k+1),'Color',z_color(k,:),"linewidth",3)
    %fprintf('loop ran %f\n',k)
end
scatter(path.Var3,path.Var2,'r',"linewidth",1,"Marker","+")
colorbar;
colormap(jet)
clim([min(survey.NumSats) max(survey.NumSats)])
%caxis([0 max(survey.NumSats)]) only uncommmetn if sat# is the same

plot(outline.GPSLongitude,outline.GPSLatitude,"k")
xlabel("Longitude (DD)")
ylabel("Latitude (DD)")
ylabel(colorbar,'Number of Satellites')
title("GNSS Satellites in Sky View")
axis equal
hold off

%% GPS Health 

numsatavg=mean(survey.NumSats);
rcvsatavg=mean(outline.NumSats);

figure; 
hold on
subplot(3,2,1);
plot(survey.Time,survey.GPSMode,"k")
xlabel("Time")
ylabel("GPS mode")
title("ASV GPS Mode")

subplot(3,2,3)
plot(survey.Time,survey.Dop,"k")
xlabel("Time")
ylabel("dop")
title("ASV Dilution of precision")

subplot(3,2,5)
plot(survey.Time,survey.NumSats,"k")
yline(numsatavg,"b")
xlabel("Time")
ylabel("# of sats")
title("ASV # of Satellites")

subplot(3,2,2)
plot(outline.Time,outline.GPSMode,"k")
xlabel("Time")
ylabel("GPS mode")
title("RCV GPS Mode")


subplot(3,2,4)
plot(outline.Time,outline.Dop,"k")
xlabel("Time")
ylabel("dop")
title("RCV Dilution of precision")

subplot(3,2,6)
plot(outline.Time,outline.NumSats,"k")
yline(rcvsatavg,"b")
xlabel("Time")
ylabel("# of sats")
title("RCV # of Satellites")

sgtitle("GPS Health")

hold off


