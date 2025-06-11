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



survey=readtable("TestData\Test_8\test8_surveycorr.txt");

outline=readtable("TestData\Test_8\test8_outline.txt");

path=readtable("TestData\Test_8\test8_path.txt");

wind=readtable("TestData\Test_8\test8_wind.txt");


%% DOL
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
sgtitle("Distance From Waypoints")
hold off


survey.DOL= abs(survey.DOL);
z_norm = (survey.DOL - min(survey.DOL)) / ((max(abs(survey.DOL))));
z_color = interp1(linspace(0,1,size(jet,1)),jet,z_norm);

figure;
hold on
for k=1:length(survey.GPSLongitude)-1
    plot(survey.GPSLongitude(k:k+1),survey.GPSLatitude(k:k+1),'Color',z_color(k,:),"linewidth",3)
    fprintf('loop ran %f\n',k)
end
scatter(path.Var3,path.Var2,'r',"linewidth",1,"Marker","+")
colorbar;
colormap(jet)
plot(outline.GPSLongitude,outline.GPSLatitude,"k")
xlabel("Longitude")
ylabel("Latitude")
sgtitle("Waypoint Deveation")

%DOL Distance off Line
%DBL Distance Begining Line
%add error for waypoints vs position
hold off



%% Speed Plot

surveyspeed= 1.64; %ft/s    .5m/s
avgss=mean(survey.Speed);
avgouts=mean(outline.Speed);

figure;
hold on
subplot(2,1,1);
plot(survey.Time,survey.Speed,"Color","k");
yline(surveyspeed,"r-")
yline(avgss,"b-")
legend ("ASV Survey Speed","Set Survey Speed","Avg Survey Speed")
subplot(2,1,2)
plot(outline.Time,outline.Speed,"Color","k")
yline(avgouts,"b-")
legend("RCV Survey Speed","Avg Survey Speed")
sgtitle("USV Survey Speed")

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


%% ASV Stats
rawerror= abs(survey.RawDepth1-survey.RawDepth2);
correrror= abs(survey.Corr_Depth1-survey.Corr_Depth2);
lowerror= abs(survey.RawDepth1-survey.Corr_Depth1);
higherror= abs(survey.RawDepth2-survey.Corr_Depth2);

figure;
hold on
plot(survey.Time, survey.Pitch)
plot(survey.Time, survey.Roll)
plot(survey.Time, survey.COG-180)
legend("Pitch","Roll","Yaw")
title("IMU measurments")
xlabel("time")
ylabel("Angle Degrees")
hold off

figure;
hold on 
plot(survey.Time, survey.Corr_Depth1,"color","b","LineWidth",2)
plot(survey.Time, survey.Corr_Depth2,"color","r","LineWidth",2)
plot(survey.Time, survey.RawDepth1,"color","[0,.5,1]","LineStyle","--")
plot(survey.Time, survey.RawDepth2,"color","[1,.2,0]","LineStyle","--")
plot(survey.Time, rawerror,"color","k")
%hold off
%plot(survey.Time, correrror)
legend("High Freq Corr","Low Freq Corr","High Freq Raw","Low Freq Raw", "Offset")
title("Corrected Dual Frequency Echosounder")
xlabel("Time")
ylabel("Depth (m)")


% plot(survey.Time, lowerror)
% plot(survey.Time, higherror)
% rectoactively change raw data to inclued corrections???

hold off




