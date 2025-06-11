clc
clear
close all

%% ROI Survey

% ROI survey location 
outline=readtable("TestData\Test_12\test12_outline.txt");

long0 = outline.GPSLongitude;
lat0 = outline.GPSLatitude;
alt0=outline.GPSElevation;


refLat= 35.2271;  % Latitude of Charlotte, NC
refLon = -80.8431; % Longitude of Charlotte, NC
refAlt = 0;        % Altitude of Charlotte, NC 

% DD -> meters (m)
enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

easting= enu(:,1);
northing= enu(:,2);
alt=enu(:,3);



% Draw a polygon and get its vertices

load("ROI.mat") %roiVertices

% figure;
% plot(easting, northing);
% title('Draw a polygon for the region of interest');
% 
% roi = drawpolygon;
% roiVertices = roi.Position;
% roiVertices(size(roiVertices,1)+1,1)=roiVertices(1,1);
% roiVertices(size(roiVertices,1),2)=roiVertices(1,2);
% 
% close
%polygons = covergeDecomposition(roiVertices);
cs = uavCoverageSpace(Polygons=roiVertices,UseLocalCoordinates=true);
ReferenceHeight = 0;
cs.UnitWidth = 10;
show(cs);

setCoveragePattern(cs,1,SweepAngle=90)
cp = uavCoveragePlanner(cs,Solver="Exhaustive");

takeoff= [-6786.75115207373	14675.7142857143 0];
%landing = [-6786.75115207373	14675.7142857143 0];

[wp,soln]=plan(cp,takeoff);
hold on
plot(wp(:,1),wp(:,2),LineWidth=1.5);
plot(takeoff(1),takeoff(2),MarkerSize=25,Marker=".")
title("Latta Nature coverage plan")
legend("","","Path","Takeoff/Landing")
xlabel('easting (m)')
ylabel('northing (m)')
hold off


% plot(roiVertices(:,1),roiVertices(:,2))
% legend('USV Outline')
% xlabel('easting (m)')
% ylabel('northing (m)')

