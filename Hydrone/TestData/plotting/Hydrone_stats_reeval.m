%% test code prep %%

%% Notes
%change x to lat and y to long

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

survey1=readtable("TestData\Test_9\Test9_survey_1.txt");
survey2=readtable("TestData\Test_10\Test10_survey.txt");
survey3=readtable("TestData\Test_12\Test12_survey_1.txt");
survey4=readtable("TestData\Test_18\10line10ft.txt");
survey5=readtable("TestData\Test_8\Test8_survey.txt");
survey6=readtable("TestData\Test_18\20line5ft.txt");
survey7=readtable("TestData\Test_12\Test12_survey_2.txt");
survey8=readtable("TestData\Test_9\Test9_survey_2.txt");

survey = vertcat(survey1, survey2, survey3,survey4,survey5,survey6,survey7,survey8);

outline=readtable("TestData\Test_17\Test17_outline.txt");

path=readtable("TestData\Test_17\Test17_path.txt");

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

%% outline convert GPS to (m)

lat0=outline.GPSLatitude;
long0=outline.GPSLongitude;
alt0=zeros(1,size(outline.GPSLatitude,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

outline.GPSLatitude=enu(:,1)-minlat;
outline.GPSLongitude=enu(:,2)-minlon;

%% path convert GPS to (m)

lat0=path.Var2;
long0=path.Var3;
alt0=zeros(1,size(path.Var3,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

path.Var2=enu(:,1)-minlat;
path.Var3=enu(:,2)-minlon;




%% DOL
distanceoffline = survey.DOL;
maxdis=max(abs(distanceoffline));
%buffer for max DOL calculation
if maxdis > 5 
    maxdis=5;
end

for p=1:length(survey.DOL)
    if survey.DOL(p)>maxdis || survey.DOL(p)<-maxdis
       survey.DOL(p) = NaN;
    end
end



%% Roll Pitch Yaw

distanceoffline = survey.DOL;
maxdis=max(abs(distanceoffline));
%buffer for max DOL calculation
if maxdis > 10 
    maxdis=10;
end

for p=1:length(survey.DOL)
    if survey.Pitch(p)>maxdis || survey.Pitch(p)<-maxdis
       survey.Pitch(p) = NaN;
    end
end
if maxdis > 10 
    maxdis=10;
end

for p=1:length(survey.DOL)
    if survey.Roll(p)>maxdis || survey.Roll(p)<-maxdis
       survey.Roll(p) = NaN;
    end
end



%% Speed Plot

surveyspeed= 2; %ft/s    .5m/s
avgss=mean(survey.Speed);
avgouts=mean(outline.Speed);




%% Depth
rawerror= abs(survey.RawDepth1-survey.RawDepth2);
correrror= abs(survey.Corr_Depth1-survey.Corr_Depth2);
lowerror= abs(survey.RawDepth1-survey.Corr_Depth1);
higherror= abs(survey.RawDepth2-survey.Corr_Depth2);













%% add plot for gps health


survey.NumSats= abs(survey.NumSats);
survey.NumSats=survey.NumSats./max(survey.NumSats);
z_norm = ((survey.NumSats) - min(survey.NumSats)) / ((max(abs(survey.NumSats))- min(survey.NumSats)));
cmap=jet(256);


%% GPS Health 

numsatavg=mean(survey.NumSats);



% Plot histogram
figure;
survey.DOL = (survey.DOL)./3.281;
histogram(survey.DOL, 'Normalization', 'pdf', 'FaceColor', 'b', 'EdgeColor', 'k');
hold on;

title('Histogram of Distance off line stats',FontSize=14);
xlabel('DOL (ft)',FontSize=12);
ylabel('Density',FontSize=12);


% Calculate statistics
meanVal = mean(survey.DOL, 'omitnan');
medianVal = median(survey.DOL, 'omitnan');
stdDev = std(survey.DOL, 'omitnan');

% Plot mean and median lines
xline(meanVal, '--r', 'LineWidth', 2, 'Label', sprintf('Mean = %.2f', meanVal),FontSize=12);
%xline(medianVal, '--g', 'LineWidth', 2, 'Label', sprintf('Median = %.2f', medianVal));

% Add text box for stats
text(max(xlim)*0.5, max(ylim)*0.7, ...
    sprintf('Std Dev = %.2f\nn = %d', stdDev, numel(survey.DOL)), ...
    'FontSize', 12, 'BackgroundColor', 'w', 'EdgeColor', 'k');

hold off;

% Plot histogram
figure;
histogram(survey.Roll, 'Normalization', 'pdf', 'FaceColor', 'b', 'EdgeColor', 'k');
hold on;

title('Histogram of Roll stats');
xlabel('Roll (Deg)');
ylabel('Density');
hold off

figure;
histogram(survey.Pitch, 'Normalization', 'pdf', 'FaceColor', 'b', 'EdgeColor', 'k');
hold on;
title('Histogram of Pitch stats');
xlabel('Pitch (Deg)');
ylabel('Density');
hold off



figure;
histogram(rawerror, 'Normalization', 'pdf', 'FaceColor', 'b', 'EdgeColor', 'k');
hold on;

title('Histogram of High/Low freq Depth Difference stats');
xlabel('Depth Difference (ft)');
ylabel('Density');

% Calculate statistics
meanVal = mean(rawerror, 'omitnan');
medianVal = median(rawerror, 'omitnan');
stdDev = std(rawerror, 'omitnan');

% Plot mean and median lines
xline(meanVal, '--r', 'LineWidth', 2, 'Label', sprintf('Mean = %.2f', meanVal));
xline(medianVal, '--g', 'LineWidth', 2, 'Label', sprintf('Median = %.2f', medianVal));

% Add text box for stats
text(max(xlim)*0.5, max(ylim)*0.7, ...
    sprintf('Std Dev = %.2f\nn = %d', stdDev, numel(survey.DOL)), ...
    'FontSize', 12, 'BackgroundColor', 'w', 'EdgeColor', 'k');
xlim([0,8])
hold off;

