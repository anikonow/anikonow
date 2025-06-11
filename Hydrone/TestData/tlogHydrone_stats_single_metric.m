%% test code prep %%

clear
clc
close all

%% To-Do



%% add survey file names

msgtable = mavlinktlog('Data\test26\2025-04-08 11-39-05.tlog');
survey = readmsg(msgtable);

GPS=((survey.Messages{8,1})); % GPS_RAW_INT
IMU=survey.Messages{11,1}; %ATTITUDE
Xtrackerror=survey.Messages{27,1}; % test 23=28 test 22 = 26


GPS=GPS(1:1:end,:);
gpsdata=[GPS.lat,GPS.lon,GPS.satellites_visible,GPS.eph];
gpsdata=double(gpsdata);
gpsdata=gpsdata(1000:end,:);


IMU=IMU(1:1:end,:);
imudata=[IMU.roll,IMU.pitch,IMU.yaw];
imudata=double(imudata);
imudata=imudata(1000:end,:);

Xtrackerror=Xtrackerror(1:1:end,:);
xtrackdata=[Xtrackerror.xtrack_error,Xtrackerror.wp_dist];
xtrackdata=double(xtrackdata);
xtrackdata=xtrackdata(1000:end,:);




%% convert GPS to (m)

refLat= 35.2271;  % Latitude of Charlotte, NC
refLon = -80.8431; % Longitude of Charlotte, NC
refAlt = 0;        % Altitude of Charlotte, NC 

lat0=gpsdata(:,1).*10^-7;
long0=gpsdata(:,2).*10^-7;
alt0=zeros(1,size(gpsdata,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

minlat=min(enu(:,1));
minlon=min(enu(:,2));

gpsdata(:,1)=enu(:,1)-minlat;
gpsdata(:,2)=enu(:,2)-minlon;




gpsdata1(:,3)= abs(gpsdata(:,3))./max(gpsdata(:,3));
z_norm = ((gpsdata1(:,3)) - min(gpsdata1(:,3))) / ((max(abs(gpsdata1(:,3)))- min(gpsdata1(:,3))));
cmap=jet(256);
z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);


figure;
hold on
for k=1:length(gpsdata1)-1
    plot(gpsdata(k:k+1,1),gpsdata(k:k+1,2),'Color',z_color(k,:),"linewidth",3)
    %fprintf('loop ran %f\n',k)
end
colorbar;
colormap(jet)
clim([min(gpsdata1(:,3)) max(gpsdata1(:,3))])
%caxis([0 max(survey.NumSats)]) only uncommmetn if sat# is the same

xlabel("X (m)",fontsize=12)
ylabel("Y (m)",fontsize=12)
ylabel(colorbar,'% of Satellites',fontsize=12)
title("GNSS Satellites in Sky View",fontsize=14)
axis equal
hold off



%% Roll Pitch Yaw

figure;
hold on
plot(imudata(:,1),'r',linewidth=2)
plot(imudata(:,2),'g',linewidth=2)
plot(imudata(:,3),'b',linewidth=2)
legend("Pitch","Roll","Yaw",fontsize=12)
title("IMU measurments",fontsize=14)
xlabel("Time",fontsize=12)
ylabel("Angle Degrees",fontsize=12)
hold off






figure;
hold on
plot(gpsdata(:,4).*.001,'k',linewidth=2)
xlabel('Data Points',FontSize=12)
ylabel('Horizontal Error (m)',FontSize=12)
title('GNSS Error',FontSize=14)

