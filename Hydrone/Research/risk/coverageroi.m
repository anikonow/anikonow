clc
clear
close all


%% ROI survey

%ROI survey input / data location
outline=readtable("TestData\Test_21\T.txt");

long0 = outline.GPSLongitude;
lat0 = outline.GPSLatitude;
alt0= outline.Altitude;

%DD -> meters (m)
refLat= min(lat0);  % Latitude of Charlotte, NC
refLon = min(long0); % Longitude of Charlotte, NC
refAlt = 0;        % Altitude of Charlotte, NC 


enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

easting= enu(:,1);
northing= enu(:,2);
alt=enu(:,3);

%% swath width calc
%avgdepth = mean(outline.Corr_Depth1);
avgdepth = mean(outline.Depth_m);

bw=5; %beamwidth 
%beam width spec 5degrees/26degrees
bw=bw/2;
bw= deg2rad(bw);
radius=avgdepth*tan(bw)*2;
polyroisurvey = polyshape(easting,northing);
area = sqrt(area(polyroisurvey));

swathwidth = (area * .05);
if swathwidth < 2
    swathwidth = 2;
end

swathwidth=7;

%% Define workspace
% Operator Defines Border
%load("ROI.mat") %roiVertices

figure;
plot(easting, northing, color='r',linewidth=2);
xlabel('easting (m)')
ylabel('northing (m)')
legend('ROI data')
title('Draw Region of Interest (ROI)');

roi = drawpolygon;
roiVertices = roi.Position;
roiVertices(size(roiVertices,1)+1,1)=roiVertices(1,1);
roiVertices(size(roiVertices,1),2)=roiVertices(1,2);

polyroi = polyshape(roiVertices);

close

% Opereator Defines Obsticals
obnum = input('Enter the number of obstacles in the region:');
disp(['Number of Obstacles:', num2str(obnum)])

figure;
hold on

for i= 1:obnum
    plot (easting, northing,color='r',linewidth=2)
    plot (roiVertices(:,1),roiVertices(:,2),color='k')
    title(sprintf('Draw Obstacle %d', i));

    roi = drawpolygon;
    ObsticalVertices{i} = roi.Position;
end
close;

%% Survey Obby Inflation

Buff = 2; % Buffer distance (m)
for i=1:obnum
    polygon=polyshape(ObsticalVertices{i});
    BuffObstical{i} = polybuffer(polygon,Buff);
end

newroi = polybuffer(polyroi,-Buff);
roiVertices = newroi.Vertices(:,:);




%% Decomposition
for i = 1:obnum




[minX, idx] = min(BuffObstical{1,i}.Vertices(:,1));
minY = min(BuffObstical{1,i}.Vertices(idx,2));
[maxX, idx] = max(BuffObstical{1,i}.Vertices(:,1));
maxY = min(BuffObstical{1,i}.Vertices(idx,2));

crit= [minX,minY,maxX,maxY];

lineminX = [minX,minX];
lineminY = [min(roiVertices(:,2))-2,max(roiVertices(:,2))+2];

linemaxX = [maxX,maxX];
linemaxY = [min(roiVertices(:,2)),max(roiVertices(:,2))];


cutshape1 = polyshape([0 0 minX minX],[min(roiVertices(:,2))-2 max(roiVertices(:,2))+2 max(roiVertices(:,2))+2 min(roiVertices(:,2))-2 ]);
cutshape2 = polyshape([12000 12000 maxX maxX],[min(roiVertices(:,2))-2 max(roiVertices(:,2))+2 max(roiVertices(:,2))+2 min(roiVertices(:,2))-2 ]);
cutshape3 = polyshape([minX minX maxX maxX],[minY 0 0 maxY]);
cutshape4 = polyshape([minX minX maxX maxX],[minY 10000 10000 maxY]);

[Ix1, Iy1] = polyxpoly(roiVertices(:,1), roiVertices(:,2), lineminX, lineminY);
[Ix2, Iy2] = polyxpoly(roiVertices(:,1), roiVertices(:,2), linemaxX, linemaxY);

D = polyshape({roiVertices(:,1)},{roiVertices(:,2)});
cell{1,i*3-2} = intersect(D,cutshape1);

cell{1,i*3-1} = intersect(D,cutshape3);
cell{1,i*3} = intersect(D,cutshape4);

cell{1,i*3+1} = intersect(D,cutshape2);



%pgon= subtract(pgon,lineminX,lineminY);
cell{1,i*3-1} = subtract(cell{1,i*3-1},BuffObstical{1,i});
cell{1,i*3} = subtract(cell{1,i*3},BuffObstical{1,i});


hold on
plot(polyroi,"FaceColor",'k','FaceAlpha',.1)
for j = i*3-2:i*3
    plot(cell{1,j})
    if i == obnum
        plot(cell{1,i*3+1})
    end
end
fprintf('Loop ran %d times \n', i);

roiVertices = cell{1,i*3+1}.Vertices(:,:);
end

%polyout2 = subtract(poly2,poly1);



%polygons = coverageDecomposition(roiVertices);
for i= 1:size(cell,2)
polygons{1,i} = cell{1,i}.Vertices;
end

cs = uavCoverageSpace(Polygons=polygons,UseLocalCoordinates=true);
ReferenceHeight = 0;
cs.UnitWidth = swathwidth;
cs.UnitLength = .1;

% setCoveragePattern(cs,1,SweepAngle=90)
% cp = uavCoveragePlanner(cs,Solver="Exhaustive");

takeoff= [roiVertices(1,:),0];
landing = [roiVertices(1,:),0];
% 
% show(cs)


%% Solver
%cpeExh = uavCoveragePlanner(cs,Solver="Exhaustive");
cpMin = uavCoveragePlanner(cs,Solver="MinTraversal");

% cpeExh.SolverParameters.VisitingSequence = [2 3 1 5 4];
%cpMin.SolverParameters.VisitingSequence = [1 2 4 5 7 8 10 9 6 3];

%[wptsExh,solnExh] = plan(cpeExh,takeoff,landing);
[wptsMin,solnMin] = plan(cpMin,takeoff,landing);

%% waypoint filter %%

% for i = 1:size(wptsMin,1)-1
%     hold on
%     lineX = [wptsMin(i,1), wptsMin(i+1,1)]; % X-coordinates of the line
%     lineY = [wptsMin(i,2), wptsMin(i+1,2)]; % Y-coordinates of the line
%     [intersectX, intersectY] = polyxpoly(lineX, lineY, BuffObstical{1,1}.Vertices(:,1),BuffObstical{1,1}.Vertices(:,2));
% 
%     if ~isempty(intersectX)
%     disp('The line intersects the obstacle.');
%     plot(lineX,lineY,'r')
%     else
%     plot(lineX,lineY,'k')
%     end
% 
% fprintf('loop ran %d times\n',i)
% 
% end
% hold off
offset=0;
wptsMin(end, :) = [];
wptsMin(1,:) = [];
pp = wptsMin;

BuffObstical{1,obnum+1} = newroi;

for ii = 1:obnum+1

for i = 1:size(wptsMin,1)-1
    hold on
    move=0;
    lineX = [wptsMin(i,1), wptsMin(i+1,1)]; % X-coordinates of the line
    lineY = [wptsMin(i,2), wptsMin(i+1,2)]; % Y-coordinates of the line
    [intersectX, intersectY] = polyxpoly(lineX, lineY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    
    if ~isempty(intersectX)
    fprintf('The line intersects the obstacle. \n');
    plot(lineX,lineY,'r','LineWidth',1)
    point = [sum(lineX) , sum(lineY)]./2;
    


    while move < 100
    move= move+.1;
    fprintf('move is equal too %f.2 \n',move)
    %Right move
    newpoint = point+[move,0];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end

    %Right Up move
    newpoint = point+[move,move];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end


    
    %Left move
    newpoint = point+[-move,0];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end

    %Left Up move 
    newpoint = point+[-move,move];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end

    %up move
    newpoint = point+[0,move];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end

    %Left down
    newpoint = point+[-move,-move];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end

    % right down move
    newpoint = point+[move,-move];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end

    % down move
    newpoint = point+[0,-move];
    AX = [lineX(1,1) newpoint(1,1)];
    AY = [lineY(1,1) newpoint(1,2)];

    BX = [newpoint(1,1) lineX(1,2)];
    BY = [newpoint(1,2) lineY(1,2)];

    [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));

    if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    newwaypoint(i+offset+1,:) = newpoint;
    offset=offset+1;
    break
    end

    end

    else
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)]; 
    end
    
%plot(lineX,lineY,'k--')
    
   

fprintf('loop ran %d times\n',i)

end
newwaypoint(i+offset+1,:) = [wptsMin(i+1,1),wptsMin(i+1,2)]; 
wptsMin = newwaypoint;

end



wptsMin(:,3) = zeros(size(wptsMin,1),1);
plot(wptsMin(:,1),wptsMin(:,2),'k')
%plot(pp(:,1),pp(:,2),'r--')
hold off

figure; 
hold on
plot(polyroi,'FaceColor','k','FaceAlpha',.4)
plot(newroi,'FaceColor','b','FaceAlpha',.1)

plot(wptsMin(:,1),wptsMin(:,2),'k')

for i = 1:length(ObsticalVertices)
    vertices = ObsticalVertices{i}; % Extract vertices from the cell
    buffverts = BuffObstical{i}.Vertices;
    fill([buffverts(:,1); buffverts(1,1)], [buffverts(:,2); buffverts(1,2)], 'k','FaceAlpha',.4,'EdgeColor','k'); 
    fill([vertices(:,1); vertices(1,1)], [vertices(:,2); vertices(1,2)], 'r','FaceAlpha',.2,'EdgeColor','k')
end






%% UTM to DD and sat plot
WGS1984 = enu2lla([wptsMin(:,1), wptsMin(:,2), wptsMin(:,3)], [refLat, refLon, refAlt], 'ellipsoid');
WGS1984 = WGS1984(1:size(WGS1984,1)-1,:);
waypoint = [WGS1984(:,1),WGS1984(:,2)];


figure
hold on 
gx = geoaxes;
geobasemap(gx,'satellite')
geoplot(gx,(WGS1984(:,1)),WGS1984(:,2),lat0,long0,'r-')

hold off


%% waypoint printer



% fileID = fopen('waypointnew.L84','w');
% fprintf(fileID,"LNS 1\n");
% fprintf(fileID,"Lin %d\n",size(waypoint,1));
% for i = 1:size(waypoint,1)
% fprintf(fileID,"PTS %.10f %.10f\n",waypoint(i,:));
% end
% fprintf(fileID,"LLN 1\n");
% fprintf(fileID,"EOL");


%writematrix(waypoint, 'waypointtest.txt');

% plot(roiVertices(:,1),roiVertices(:,2))
% legend('USV Outline')
% xlabel('easting (m)')
% ylabel('northing (m)')

%% waypoint printer for .waypoint

% File name for the output .waypoints file
filename = 'Test21_waypoints.waypoints';

% Open the file for writing
fileID = fopen(filename, 'w');

% Write the header
fprintf(fileID, 'QGC WPL 110\n'); 

% Loop through each waypoint and write it to the file
for i = 1:size(WGS1984, 1)
    fprintf(fileID, '%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.6f\t%.6f\t%d\t%d\n', ...
        i-1, i==1, 3, 16,0, 0, 0, 0, WGS1984(i,1),WGS1984(i,2),100,1)
end

% Close the file
fclose(fileID);

disp(['Waypoints written to ', filename]);

%% WAYPOINT SOLVER PLOT
% % figure
% % hold on
% % show(cs);
% % title("Exhaustive Solver Algorithm")
% % 
% % 
% % 
% % % [wp,soln]=plan(cp,takeoff);
% % % hold on
% % 
% % %plot(wptsExh(:,1),wptsExh(:,2),LineWidth=1.5);
% % plot(wptsMin(:,1),wptsMin(:,2),LineWidth=1.5,LineStyle="--");
% % plot(takeoff(1),takeoff(2),MarkerSize=25,Marker=".")
% % 
% % hold on
% % for i = 1:length(ObsticalVertices)
% %     vertices = ObsticalVertices{i}; % Extract vertices from the cell
% %     buffverts = BuffObstical{i}.Vertices;
% %     fill([buffverts(:,1); buffverts(1,1)], [buffverts(:,2); buffverts(1,2)], 'k','FaceAlpha',.4,'EdgeColor','k'); 
% %     fill([vertices(:,1); vertices(1,1)], [vertices(:,2); vertices(1,2)], 'r','FaceAlpha',.2,'EdgeColor','k')
% % end
% % 
% % plot(Ix1,Iy1)
% % plot(Ix2,Iy2)
% % scatter(crit(1,1),crit(1,2))
% % scatter(crit(1,3),crit(1,4))
% % 
% % title("Coverage Plan")
% % xlabel('Easting (m)')
% % ylabel('Northing (m)')
% % hold off