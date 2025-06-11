%% test code prep %%

%% Notes
% use for data plotting for tests 1-17 Hypack data collection
% test 18 has its own code
% tests 20-end used opensource data collection



clear
clc
close all

%% To-Do
%make plots paper ready


%% add survey file names

survey1=readtable("Data\test26\surveyB1_high.txt"); %high
survey2=readtable("Data\test26\surveyB1_low.txt");%low freq

survey = addvars(survey1, survey2.Depth_m, 'NewVariableNames', 'Depth_m2');

survey.Corr_Depth1=survey.Depth_m;
survey.Corr_Depth2=survey.Depth_m2;
survey.Time=survey.GNSS_Time;
%survey = survey(400:2500,:);

outline=readtable("Data\test26\outlineA1.txt");
%outline=readtable("Data\test22\outlineA1_low.txt");

%outline = addvars(outline, outline1.Depth_m, 'NewVariableNames', 'Depth_m2');

outline.Corr_Depth1=outline.Depth_m;
%outline.Corr_Depth2=outline.Depth_m2;
outline.Time=outline.GNSS_Time;

path=readtable("Data\test26\pathB.txt");

%wind=readtable("TestData\Test_9\test9_wind.txt");

%% survey convert GPS to (m)

refLat= 35.2271;  % Latitude of Charlotte, NC
refLon = -80.8431; % Longitude of Charlotte, NC
refAlt = 0;        % Altitude of Charlotte, NC 

lat0=survey.GPSLatitude;
long0=survey.GPSLongitude;
alt0=zeros(1,size(survey.GPSLatitude,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

minlat=min(enu(:,1));
minlon=min(enu(:,2));

survey.GPSLatitude=enu(:,1)-minlat;
survey.GPSLongitude=enu(:,2)-minlon;


survey=survey(630:5:end-650,:);
%% outline convert GPS to (m)

lat0=outline.GPSLatitude*10^-7;
long0=outline.GPSLongitude*10^-7;
alt0=zeros(1,size(outline.GPSLatitude,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

outline.GPSLatitude=enu(:,1)-minlat;
outline.GPSLongitude=enu(:,2)-minlon;

%% path convert GPS to (m)

lat0=path.Var9;
long0=path.Var10;
alt0=zeros(1,size(path.Var10,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

path.Var9=enu(:,1)-minlat;
path.Var10=enu(:,2)-minlon;




% % %% DOL
% % 
% % distanceoffline = survey.DOL;
% % maxdis=max(abs(distanceoffline));
% % %buffer for max DOL calculation
% % if maxdis > 5 
% %     maxdis=5;
% % end
% % 
% % for p=1:length(survey.DOL)
% %     if survey.DOL(p)>maxdis || survey.DOL(p)<-maxdis
% %        survey.DOL(p) = 0;
% %     end
% % end
% % 
% % 
% % figure;
% % hold on
% % plot(survey.Time,survey.DOL,"k",linewidth=2)
% % yline(0,"r-",linewidth=2)
% % xlabel("Time",FontSize=12)
% % ylabel("XTE (m)",FontSize=12)
% % title("Cross-Track Error",FontSize=14)
% % 
% % figure;
% % hold on
% % plot(outline.GPSLatitude,outline.GPSLongitude,"k",linewidth=1)
% % survey.DOL= abs(survey.DOL);
% % z_norm = (survey.DOL - min(survey.DOL)) / ((max(abs(survey.DOL))));
% % z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);
% % 
% % for k=1:length(survey.GPSLongitude)-1
% %     plot(survey.GPSLatitude(k:k+1),survey.GPSLongitude(k:k+1),'Color',z_color(k,:),"linewidth",3)
% %     %fprintf('loop ran %f\n',k)
% % end
% % scatter(path.Var2,path.Var3,'k',"linewidth",1,"Marker","+",linewidth=2)
% % colorbar;
% % colormap(jet)
% % 
% % clim([min(survey.DOL) 2])
% % xlabel("X-axis (m)",FontSize=12)
% % ylabel("Y-axis (m)",FontSize=12)
% % xlabel(colorbar, 'Distance (m)',FontSize=12);
% % title("Waypoint Deviation",FontSize=14)
% % axis equal




%figure;
% plot(survey.Time,survey.DBL,"k")
% title("Distance from Begining Line")
% xlabel("Time")
% ylabel("Distance (m)")
% sgtitle("Distance From Waypoints")
% hold off


% % %% Roll Pitch Yaw
% % 
% % figure;
% % hold on
% % plot(survey.Time, survey.Pitch,'r',linewidth=2)
% % plot(survey.Time, survey.Roll,'g',linewidth=2)
% % plot(survey.Time, survey.COG-180,'b',linewidth=2)
% % legend("Pitch","Roll","Yaw",fontsize=12)
% % title("IMU measurments",fontsize=14)
% % xlabel("Time",fontsize=12)
% % ylabel("Angle Degrees",fontsize=12)
% % hold off

% % 
% % %% Speed Plot
% % 
% % surveyspeed= 1.02; %ft/s    .5m/s
% % avgss=mean(survey.Speed);
% % avgouts=mean(outline.Speed);
% % 
% % figure;
% % hold on
% % plot(survey.Time,survey.Speed,"Color","k",linewidth=2);
% % yline(surveyspeed,"r-",linewidth=2)
% % yline(avgss,"b-",linewidth=2)
% % ylabel("Speed (m/s)",fontsize=12)
% % xlabel("Time",fontsize=12)
% % legend ("ASV Survey Speed","Set Survey Speed","Avg Survey Speed",fontsize=12)
% % title('ASV Survey Speed',fontsize=14)
% % 
% % figure;
% % hold on
% % plot(outline.Time,outline.Speed,"Color","k",linewidth=2)
% % yline(avgouts,"b-",linewidth=2)
% % ylabel("Speed (m/s)",fontsize=12)
% % xlabel("Time",fontsize=12)
% % legend("RCV Survey Speed","Avg Survey Speed",fontsize=12)
% % title("USV Survey Speed",fontsize=14)




%% Depth
%rawerror= abs(survey.RawDepth1-survey.RawDepth2);
correrror= abs(survey.Corr_Depth1-survey.Corr_Depth2);
%lowerror= abs(survey.RawDepth1-survey.Corr_Depth1);
%higherror= abs(survey.RawDepth2-survey.Corr_Depth2);


figure;
hold on 
plot([1:size(survey.Corr_Depth1,1)], survey.Corr_Depth1,"color","b","LineWidth",2)
%plot([1:size(survey.Corr_Depth1,1)], survey.Corr_Depth2,"color","r","LineWidth",2)
% plot(survey.Time, survey.RawDepth1,"color","[0,.5,1]","LineStyle","--",linewidth=2)
% plot(survey.Time, survey.RawDepth2,"color","[1,.2,0]","LineStyle","--",linewidth=2)
% plot(survey.Time, rawerror,"color","k",linewidth=2)
%hold off
%plot(survey.Time, correrror)
legend("High Freq",fontsize=12)
title("Dual Frequency Echosounder Data",fontsize=14)
xlabel("Time")
ylabel("Depth (m)")

hold off


survey.Corr_Depth2= abs(survey.Corr_Depth2);
z_norm = ((survey.Corr_Depth2) - min(survey.Corr_Depth2)) / ((max(abs(survey.Corr_Depth2))- min(survey.Corr_Depth2)));
cmap=jet(256);

%z_norm=ones(363,1); %only uncomment if #of sats is same thru test

z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);

% outline.Corr_Depth2= abs(outline.Corr_Depth2);
% z1_norm = ((outline.Corr_Depth2) - min(outline.Corr_Depth2)) / ((max(abs(outline.Corr_Depth2))- min(outline.Corr_Depth2)));

% z1_color = interp1(linspace(0,1,size(jet,1)),jet,z1_norm);

figure;

hold on
plot(outline.GPSLatitude(:,1),outline.GPSLongitude(:,1),'Color','k',"linewidth",1)

for k=1:length(survey.GPSLongitude)-1
    plot(survey.GPSLatitude(k:k+1),survey.GPSLongitude(k:k+1),'Color',z_color(k,:),"linewidth",3)
    %fprintf('loop ran %f\n',k)
end



scatter(path.Var9,path.Var10,'k',"linewidth",1,"Marker","+",linewidth=2)
colorbar;
colormap(jet)
clim([min(survey.Corr_Depth2) max(survey.Corr_Depth2)])
%caxis([0 max(survey.NumSats)]) only uncommmetn if sat# is the same


title("Low Frequency Depth",fontsize=14)
xlabel("X-axis (m)",fontsize=12)
ylabel("Y-axis (m)",fontsize=12)
ylabel(colorbar,"Depth (m)",fontsize=12)
axis equal
hold off


survey.Corr_Depth1= abs(survey.Corr_Depth1);
z1_norm = ((survey.Corr_Depth1) - min(survey.Corr_Depth1)) / ((max(abs(survey.Corr_Depth1))- min(survey.Corr_Depth1)));
cmap=jet(256);

%z_norm=ones(363,1); %only uncomment if #of sats is same thru test

z1_color = interp1(linspace(0,1,size(jet,1)),jet,z1_norm);

% outline.Corr_Depth1= abs(outline.Corr_Depth1);
% z1_norm = ((outline.Corr_Depth1) - min(outline.Corr_Depth1)) / ((max(abs(outline.Corr_Depth1))- min(outline.Corr_Depth1)));
% 
% z1_color = interp1(linspace(0,1,size(jet,1)),jet,z1_norm);



figure;
hold on
plot(outline.GPSLatitude(:,1),outline.GPSLongitude(:,1),'Color','k',"linewidth",1)

for k=1:length(survey.GPSLongitude)-1
    plot(survey.GPSLatitude(k:k+1),survey.GPSLongitude(k:k+1),'Color',z1_color(k,:),"linewidth",3)
    %fprintf('loop ran %f\n',k)
end



scatter(path.Var9,path.Var10,'k',"linewidth",1,"Marker","+",linewidth=2)
colorbar;
colormap(jet)
clim([min(survey.Corr_Depth1) max(survey.Corr_Depth1)])
%caxis([0 max(survey.NumSats)]) only uncommmetn if sat# is the same


title("High Frequency Depth",fontsize=14)
xlabel("X-axis (m)",fontsize=12)
ylabel("Y-axis (m)",fontsize=12)
ylabel(colorbar,"Depth (m)",fontsize=12)
axis equal
hold off







%% add plot for gps health


% survey.NumSats= abs(survey.NumSats);
% survey.NumSats=survey.NumSats./max(survey.NumSats);
% z_norm = ((survey.NumSats) - min(survey.NumSats)) / ((max(abs(survey.NumSats))- min(survey.NumSats)));
% cmap=jet(256);
% 
% %z_norm=ones(363,1); %only uncomment if #of sats is same thru test
% 
% z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);
% 
% 
% figure;
% hold on
% 
% plot(outline.GPSLatitude,outline.GPSLongitude,"k",linewidth=1)
% 
% for k=1:length(survey.GPSLongitude)-1
%     plot(survey.GPSLatitude(k:k+1),survey.GPSLongitude(k:k+1),'Color',z_color(k,:),"linewidth",3)
%     %fprintf('loop ran %f\n',k)
% end
% scatter(path.Var9,path.Var10,'k',"linewidth",1,"Marker","+",linewidth=2)
% colorbar;
% colormap(jet)
% clim([min(survey.NumSats) max(survey.NumSats)])
% %caxis([0 max(survey.NumSats)]) only uncommmetn if sat# is the same
% 
% 
% xlabel("X-axis (m)",fontsize=12)
% ylabel("Y-axis (m)",fontsize=12)
% ylabel(colorbar,'% of Satellites',fontsize=12)
% title("GNSS Satellites in Sky View",fontsize=14)
% axis equal
% 
% hold off
% 
% %% GPS Health 
% 
% numsatavg=mean(survey.NumSats);
% rcvsatavg=mean(outline.NumSats);
% 
% figure;
% hold on
% plot(survey.Time,survey.GPSMode,"k",linewidth=2)
% xlabel("Time",fontsize=12)
% ylabel("GPS mode",fontsize=12)
% title("ASV GPS Mode",fontsize=14)
% 
% figure;
% hold on
% plot(survey.Time,survey.Dop,"k",linewidth=2)
% xlabel("Time",fontsize=12)
% ylabel("dop",fontsize=12)
% title("ASV Dilution of precision",fontsize=14)
% 
% figure;
% hold on
% plot(survey.Time,survey.NumSats,"k",linewidth=2)
% yline(numsatavg,"b",linewidth=2)
% xlabel("Time",fontsize=12)
% ylabel("# of sats",fontsize=12)
% title("ASV # of Satellites",fontsize=14)
% 
% figure;
% hold on
% plot(outline.Time,outline.GPSMode,"k",linewidth=2)
% xlabel("Time",fontsize=12)
% ylabel("GPS mode",fontsize=12)
% title("RCV GPS Mode",fontsize=14)
% 
% 
% figure;
% hold on
% plot(outline.Time,outline.Dop,"k",linewidth=2)
% xlabel("Time",fontsize=12)
% ylabel("dop",fontsize=12)
% title("RCV Dilution of precision",fontsize=14)
% 
% figure;
% hold on
% plot(outline.Time,outline.NumSats,"k",linewidth=2)
% yline(rcvsatavg,"b",linewidth=2)
% xlabel("Time",fontsize=12)
% ylabel("Number of Satellites",fontsize=12)
% title("RCV # of Satellites",fontsize=14)
% hold off
% 
% 
Y = min(survey.GPSLongitude):.5:max(survey.GPSLongitude);
X = min(survey.GPSLatitude):.5:max(survey.GPSLatitude);

[X, Y] = meshgrid(X,Y);

Xq = [X(:), Y(:)]; 
data = [survey.GPSLatitude,survey.GPSLongitude]; 

gprMdl = fitrgp(data, survey.Corr_Depth1, 'KernelFunction', 'squaredexponential');

[z,zcov] = predict(gprMdl, Xq);
z = reshape(z, size(X));
zcov = reshape(zcov, size(X));
z=z; % take out 
zcov=zcov;



figure
hold on
surf(X,Y,-z,'EdgeColor','none')
plot3(survey.GPSLatitude,survey.GPSLongitude,-0.4*ones(size(survey.GPSLatitude)),linewidth=2,Color='k')
axis equal
xlabel('X (m)',fontsize=12)
ylabel('Y (m)',fontsize=12)

title('Survey Bathymetry')
zlabel('Depth (m)')
view(2)
colorbar
ylabel(colorbar,'Depth (m)',fontsize=12)
% % filename = 'spinning_plot.gif';
% % 
% % nFrames = 120;
% % viewAngle = get(gca, 'View');
% % 
% % for i = 1:nFrames
% % 
% %     view(viewAngle(1) + i * 3, viewAngle(2));
% % 
% %     drawnow
% % 
% %     frame = getframe(gcf);
% %     img = frame2im(frame);
% %     [imind, cm] = rgb2ind(img, 256);
% % 
% %     if i == 1
% %         imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
% %     else
% %         imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
% %     end
% % end