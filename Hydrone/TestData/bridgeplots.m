clc
clear
close all

%% bridge inspections
%test 17, 21, 22



survey=readtable("Data\test17\surveyB1.txt");

refLat= 35.2271;  % Latitude of Charlotte, NC
refLon = -80.8431; % Longitude of Charlotte, NC
refAlt = 0;        % Altitude of Charlotte, NC 

lat0=survey.GPSLatitude;
long0=survey.GPSLongitude;
alt0=zeros(1,size(survey.GPSLatitude,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

minlat=min(enu(:,1));
minlon=min(enu(:,2));

test17lat=enu(:,1)-minlat;
test17lon=enu(:,2)-minlon;

rotationangle=-45;
rotationangle=deg2rad(rotationangle);

R = [cos(rotationangle), -sin(rotationangle); sin(rotationangle), cos(rotationangle)];
rotpoint = R*[test17lon';test17lat'];

easting=rotpoint(1,:)';
northing=rotpoint(2,:)';

eastingoffset=min(easting);
northingoffset=min(northing);

test17lon = easting - eastingoffset;
test17lat = northing - northingoffset;

easting17=test17lon-14;
easting17=abs(easting17);




load('Data\Misc\test21satdata.mat')
survey21=gpsdata;
maxsat=max(survey21(:,3));
survey21(:,3)=survey21(:,3)./maxsat;
survey21(:,2)=survey21(:,2).*10^-7;
survey21(:,1)=survey21(:,1).*10^-7;

lat0=survey21(:,1);
long0=survey21(:,2);
alt0=zeros(1,size(survey21,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

minlat=min(enu(:,1));
minlon=min(enu(:,2));

test21lat=enu(:,1)-minlat;
test21lon=enu(:,2)-minlon;

rotationangle=80;
rotationangle=deg2rad(rotationangle);

R = [cos(rotationangle), -sin(rotationangle); sin(rotationangle), cos(rotationangle)];
rotpoint = R*[test21lon';test21lat'];

easting=rotpoint(1,:)';
northing=rotpoint(2,:)';

eastingoffset=min(easting);
northingoffset=min(northing);

test21lon = easting - eastingoffset;
test21lat = northing - northingoffset;

easting21=test21lat-52;
easting21=abs(easting21);











load('Data\Misc\test22satdata.mat')
survey22=gpsdata;
maxsat=max(survey22(:,3));
survey22(:,3)=survey22(:,3)./maxsat;
survey22(:,2)=survey22(:,2).*10^-7;
survey22(:,1)=survey22(:,1).*10^-7;

lat0=survey22(:,1);
long0=survey22(:,2);
alt0=zeros(1,size(survey22,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

minlat=min(enu(:,1));
minlon=min(enu(:,2));

test22lat=enu(:,1)-minlat;
test22lon=enu(:,2)-minlon;

rotationangle=-14.5;
rotationangle=deg2rad(rotationangle);

R = [cos(rotationangle), -sin(rotationangle); sin(rotationangle), cos(rotationangle)];
rotpoint = R*[test22lon';test22lat'];

easting=rotpoint(1,:)';
northing=rotpoint(2,:)';

eastingoffset=min(easting);
northingoffset=min(northing);

test22lon = easting - eastingoffset;
test22lat = northing - northingoffset;

easting22=test22lon-110;
easting22=abs(easting22);



% plot(easting22,test22lat)
% axis equal


sats17=survey.NumSats./max(survey.NumSats);


degree = 2;
coe17 = polyfit(easting17,sats17,4);
fit17 = polyval(coe17,easting17);

coe21 = polyfit(easting21,survey21(:,3),degree);
fit21 = polyval(coe21,easting21);

coe22 = polyfit(easting22,survey22(:,3),4);
fit22 = polyval(coe22,easting22);

figure;
hold on

xline(0,'k',linestyle='--',linewidth=2)
plot(easting17,fit17,'b',linewidth=2)
plot(easting21,fit21,'k',linewidth=2)
plot(easting22,fit22,'r',linewidth=2)
scatter(easting17,sats17,'b','o','MarkerEdgeAlpha',.2);

scatter(easting21,survey21(:,3),'k','o','MarkerEdgeAlpha',.05)
scatter(easting22,survey22(:,3),'r','o','MarkerEdgeAlpha',.05)

title('Bridge GNSS Degradation',fontsize=14)
xlabel('Distance from Bridge (m)',FontSize=12)
ylabel('Satellites in Sky View (%)',FontSize=12)
legend('Center of Bridge','Test 17 data','Test 21 data','Test 22 data',fontsize=12)