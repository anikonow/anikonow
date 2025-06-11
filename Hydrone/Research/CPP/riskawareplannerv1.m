clc
clear
close all


%% ROI survey

%init
cellnum=15; %cells per interpolated point
shallowcut=[-.1,.7]; %pdf intigration region
target = .95; %target accuracy 
counter = 1; %counter term for while loop
ie=0; %integral error
ObsticalVertices=[]; %init obverts

%ROI survey input / data location
outline=readtable("TestData\Test_21\Test21_outline.csv");

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


%depth=outline.Depth_m;
depth=outline.Depth_m;


%% Depth Grid
X = min(easting):.5:max(easting);
Y = min(northing):.5:max(northing);

[X, Y] = meshgrid(X,Y);

Xq = [X(:), Y(:)]; 
data = [easting,northing]; 

gprMdl = fitrgp(data, depth, 'KernelFunction', 'squaredexponential');

[z,zcov] = predict(gprMdl, Xq);
z = reshape(z, size(X));
zcov = reshape(zcov, size(X));
z=z-.3; % take out 

figure;
subplot(2,1,1)
hold on
contourf(X,Y,-z)
plot(easting,northing,'r',linewidth=2)
colorbar
xlabel('X (m)')
ylabel('Y (m)')
title('Bathy of ROI')
axis equal

subplot(2,1,2)
hold on
contourf(X,Y,zcov)
plot(easting,northing,'r',linewidth=2)
colorbar
xlabel('X (m)')
ylabel('Y (m)')
title('Var of ROI')
axis equal




p1=normcdf(shallowcut(1,1),z,zcov);
p2=normcdf(shallowcut(1,2),z,zcov);
r=p2-p1;



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


%% Define workspace
% Operator Defines Border
%load("ROI.mat") %roiVertices

figure;
hold on

levels = [1-target,1-target]; 
[M, c] = contourf(X, Y, r, levels, 'ShowText', 'on'); % Filled contours
colormap([0 1 0; 1 0.5 0; 1 0 0]); % Red (shallow), Orange (mid), Green (deep)


plot(easting, northing, color='r',linewidth=2);
axis equal
xlabel('easting (m)')
ylabel('northing (m)')
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

if obnum > 0
    for i= 1:obnum
    % contourf(X,Y,-z)
    % colormap("gray")
    % colorbar;
    
    levels = [1-target,1-target]; 
    [M, c] = contour(X, Y, r, levels, 'ShowText', 'on'); % Filled contours
    colormap([1 0 0; 1 0.5 0; 0 1 0]); % Red (shallow), Orange (mid), Green (deep)

    colorbar; % Show depth scale

    plot (easting, northing,color='r',linewidth=2)
    plot (roiVertices(:,1),roiVertices(:,2),color='k')
    title(sprintf('Draw Obstacle %d', i));
    xlabel('easting (m)')
    ylabel('northing (m)')
    axis equal
    roi = drawpolygon;
    ObsticalVertices{i} = roi.Position;
    end
end
close;


%% High risk obby

contourLines = {};
segmentIndex = 1;

i = 1;
while i < size(M, 2)
    numPoints = M(2, i); 
    xSegment = M(1, i+1:i+numPoints);
    ySegment = M(2, i+1:i+numPoints);

    contourLines{segmentIndex} = [xSegment; ySegment];
    segmentIndex = segmentIndex + 1;

    i = i + numPoints + 1;
end
iii=1;
for i = 1:size(contourLines,2)
    highriskobby{1,i}=polyshape(contourLines{1,i}');
    if overlaps(highriskobby{1,i},polyroi) == 1
        polyroi = subtract(polyroi,highriskobby{1,i});
    else
        riskobby{1,iii}=highriskobby{1,i};
        iii=iii+1;
    end
end





PATHRISK = 0;
Buff = 0;


%% ITERATIVE LOOP BEGIN



while PATHRISK == 0
    wptsMin = [];
    BuffObstical = [];
    roiVertices = [];
    cell=[];
    polygons=[];
    newwaypoint=[];



%% Survey Obby Inflation

 % Buffer distance (m)
for i=1:obnum
    polygon=polyshape(ObsticalVertices{i});
    BuffObstical{i} = polybuffer(polygon,Buff);
    OccObstical{i}=polybuffer(polygon,Buff-.12);
end

newroi = polybuffer(polyroi,-Buff); %% change -2 to buff at some point
occroi = polybuffer(polyroi,-Buff+.12);
roiVertices = [];
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

% figure;
% hold on
% plot(polyroi,"FaceColor",'k','FaceAlpha',.1)
% for j = i*3-2:i*3
%     plot(cell{1,j})
%     if i == obnum
%         plot(cell{1,i*3+1})
%     end
% end

fprintf('Loop ran %d times \n', i);

roiVertices = cell{1,i*3+1}.Vertices(:,:);
end

%polyout2 = subtract(poly2,poly1);



%polygons = coverageDecomposition(roiVertices);
if obnum > 0
    for i= 1:size(cell,2)
    polygons{1,i} = cell{1,i}.Vertices;
    end
else
    polygons{1,1} = roiVertices;
end

%% occapancy map
mapsize = [max(easting),max(northing)];
reso = 20;
map = occupancyMap(mapsize(1), mapsize(2), reso);
[xGrid, yGrid] = meshgrid(1:(1/(reso*2)):mapsize(1), 1:(1/(reso*2)):mapsize(2));
obsMask = inpolygon(xGrid, yGrid, occroi.Vertices(:,1), occroi.Vertices(:,2));
setOccupancy(map, [xGrid(obsMask), yGrid(obsMask)], 0);

if obnum > 0
    for i = 1:size(BuffObstical,2)  
    obsMask = inpolygon(xGrid, yGrid, OccObstical{1,i}.Vertices(:,1), OccObstical{1,i}.Vertices(:,2));
    setOccupancy(map, [xGrid(obsMask), yGrid(obsMask)], 1);
    end
end


cs = uavCoverageSpace(Polygons=polygons,UseLocalCoordinates=true);
ReferenceHeight = 0;
cs.UnitWidth = swathwidth;
cs.UnitLength = 1;

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
% wptsMin(end, :) = [];
% wptsMin(1,:) = [];
pp = wptsMin;
newwaypoint=[];

BuffObstical{1,obnum+1} = newroi; %idk why this is after decomposition

for ii = 1:1:obnum+1   %,obnum+1:-1:1,1:1:obnum+1] end at ~496 

for i = 1:size(wptsMin,1)-1
    
    %move=0;
    lineX = [wptsMin(i,1), wptsMin(i+1,1)]; % X-coordinates of the line
    lineY = [wptsMin(i,2), wptsMin(i+1,2)]; % Y-coordinates of the line
    [intersectX, intersectY] = polyxpoly(lineX, lineY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    
    if ~isempty(intersectX)
    fprintf('The line intersects the obstacle. \n');
    %plot(lineX,lineY,'r','LineWidth',1)

    %% new system
    startpoint= [wptsMin(i,1),wptsMin(i,2)];
    endpoint= [wptsMin(i+1,1),wptsMin(i+1,2)];
    
    prm = mobileRobotPRM(map, 400);
    map.OccupiedThreshold=.2;

    path = findpath(prm, startpoint, endpoint);
    path(end,:)=[];
    
    %newwaypoint(i+offset,:)=path;
    newwaypoint=[newwaypoint;path];
    offset=offset+size(path,1)-1;



    % % % point = [lineX(1,1) , lineY(1,1)];

    % % % while move < 100
    % % % move= move+.1;
    % % % fprintf('move is equal too %f.2 \n',move)
    % % % %Right move
    % % % newpoint = point+[move,0];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % %Right Up move
    % % % newpoint = point+[move,move];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % 
    % % % 
    % % % %Left move
    % % % newpoint = point+[-move,0];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % %Left Up move 
    % % % newpoint = point+[-move,move];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % %up move
    % % % newpoint = point+[0,move];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % %Left down
    % % % newpoint = point+[-move,-move];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % % right down move
    % % % newpoint = point+[move,-move];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % % down move
    % % % newpoint = point+[0,-move];
    % % % AX = [lineX(1,1) newpoint(1,1)];
    % % % AY = [lineY(1,1) newpoint(1,2)];
    % % % 
    % % % BX = [newpoint(1,1) lineX(1,2)];
    % % % BY = [newpoint(1,2) lineY(1,2)];
    % % % 
    % % % [intersectX1, intersectY1] = polyxpoly(AX, AY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % [intersectX2, intersectY2] = polyxpoly(BX, BY, BuffObstical{1,ii}.Vertices(:,1),BuffObstical{1,ii}.Vertices(:,2));
    % % % 
    % % % if ~isempty(intersectX1) < 1  && ~isempty(intersectX2) < 1 
    % % % newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)];
    % % % newwaypoint(i+offset+1,:) = newpoint;
    % % % offset=offset+1;
    % % % break
    % % % end
    % % % 
    % % % end

    else
    newwaypoint(i+offset,:) = [wptsMin(i,1),wptsMin(i,2)]; 
    end
    
%plot(lineX,lineY,'k--')
    
fprintf('loop ran %d times with buffer size %d \n',i,Buff)

end
newwaypoint(i+offset+1,:) = [wptsMin(i+1,1),wptsMin(i+1,2)]; 
wptsMin = newwaypoint;
newwaypoint=[wptsMin(1,1),wptsMin(1,2)];
offset=0;
end


%% Obstical plot
wptsMin(:,3) = zeros(size(wptsMin,1),1);

figure; 
subplot(3,1,1)
hold on
plot(pp(:,1),pp(:,2),'r') %non-filtered waypoints
plot(wptsMin(:,1),wptsMin(:,2),'k') %filtered waypoints

plot(polyroi,'FaceColor','k','FaceAlpha',.4)
plot(newroi,'FaceColor','b','FaceAlpha',.1)

if obnum > 0
for i = 1:length(ObsticalVertices)
    vertices = ObsticalVertices{i}; % Extract vertices from the cell
    buffverts = BuffObstical{i}.Vertices;
    fill([buffverts(:,1); buffverts(1,1)], [buffverts(:,2); buffverts(1,2)], 'k','FaceAlpha',.4,'EdgeColor','k'); 
    fill([vertices(:,1); vertices(1,1)], [vertices(:,2); vertices(1,2)], 'r','FaceAlpha',.2,'EdgeColor','k')
end
end
xlabel('X (m)')
ylabel('Y (m)')
title(sprintf('Path plan, Buffer size: %.3f', Buff));
axis equal



%% UTM to DD and sat plot
WGS1984 = enu2lla([wptsMin(:,1), wptsMin(:,2), wptsMin(:,3)], [refLat, refLon, refAlt], 'ellipsoid');
WGS1984 = WGS1984(1:size(WGS1984,1)-1,:);
waypoint = [WGS1984(:,1),WGS1984(:,2)];

% hold on
% gx = geoaxes;
% geobasemap(gx,'satellite')
% 
% geoplot(gx,(WGS1984(:,1)),WGS1984(:,2),lat0,long0,'r-')
% title('Waypoint Plot')
% hold off


%% Risk Calculations

waypoints=[wptsMin(:,1),wptsMin(:,2)];

obverts=vertcat(ObsticalVertices{:});
obverts=[obverts;polyroi.Vertices(:,:)];






[sig,interwayx,interwayy,inter,ang ] = sigmainterpol(X,Y,z,waypoints);

[collisionChance] = usvriskcalculation(X,Y,z,zcov,inter,interwayx,interwayy,sig,cellnum, shallowcut, ObsticalVertices, roiVertices);

collisionindex=[];
for g = 1:size(inter,1)
    if collisionChance(:,g) > 1-target
    collisionindex(g,:) = [1 0 0];
    else
    collisionindex(g,:) = [0 0 0];
    end
end



subplot(3,1,3)
hold on
scatter(inter(:,1), inter(:,2), 50, collisionChance(:), 'filled','MarkerEdgeColor','r');
scatter(inter(:,1), inter(:,2), 50, collisionindex(:,:));

for i = 1:length(ObsticalVertices)
    vertices = ObsticalVertices{i}; % Extract vertices from the cell
    buffverts = BuffObstical{i}.Vertices;
    fill([buffverts(:,1); buffverts(1,1)], [buffverts(:,2); buffverts(1,2)], 'k','FaceAlpha',.4,'EdgeColor','k'); 
    fill([vertices(:,1); vertices(1,1)], [vertices(:,2); vertices(1,2)], 'r','FaceAlpha',.2,'EdgeColor','k')
end

%% interplot
colormap(gca, gray); 
colorbar; 
set(gca,'ColorScale','log')
xlabel('X (m)');
ylabel('Y (m)');
title('Plot of Collision Chance');
axis equal

cost = max(collisionChance);



%% compute dis to obby
mindist=[];
for i = 1:size(inter,1)
        
    diffs = obverts - inter(i, :);
    distances = sqrt(sum(diffs.^2, 2));

       
    [min_dist, idx] = min(distances);
    closest_vertex(i,:) = obverts(idx, :);

    mindist(i)=min_dist;
end

min_mindist = min(mindist);
max_mindist = max(mindist);
grayscale_values = (mindist - min_mindist) / (max_mindist - min_mindist); 

% figure;
% hold on
% for xxx=1:size(inter,1)
%     color_val = repmat(grayscale_values(xxx),1,3);
%     plot([closest_vertex(xxx,1),inter(xxx,1)],[closest_vertex(xxx,2),inter(xxx,2)],'Color',color_val)
% end
% plot(inter(:,1),inter(:,2))
% hold off








%% Control

if cost > (1-target)
    error = cost-(1-target) ;
    
    buffer(counter) = Buff;
    costj(counter) = cost;
    
    %Buff = Buff + error^.5 * 1;
    
    ie=ie+error; % Ki init

    Buff = Buff + error*1.3 + ie*.2;
    
    
    counter=counter+1;
    %%%

    fprintf('%d',Buff)
else
    buffer(counter) = Buff;
    costj(counter) = cost;
   PATHRISK = 1;
end


ipx = (mindist < 4) & (collisionChance > 1-target);
if all(ipx == 0)
fprintf('Risk still exists, cannot be solved by obstical buffer')
PATHRISK = 1;
end

    % wptsMin(:,:) = [];
    % BuffObstical = [];
    % roiVertices = [];
    % cell=[];
    % polygons=[];
    % newwaypoint=[];


end




%% waypoint printer .L84
% This is a .L84 file
% compatable with HYPACK to be uploaded in waypoint editor 

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
% This is a .waypoint file
% compatable with MISSION PLANNING software



% % File name for the output .waypoints file
% filename = 'Test21_waypoints.waypoints';
% 
% % Open the file for writing
% fileID = fopen(filename, 'w');
% 
% % Write the header
% fprintf(fileID, 'QGC WPL 110\n'); 
% 
% % Loop through each waypoint and write it to the file
% for i = 1:size(WGS1984, 1)
%     fprintf(fileID, '%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.6f\t%.6f\t%d\t%d\n', ...
%         i-1, i==1, 3, 16,0, 0, 0, 0, WGS1984(i,1),WGS1984(i,2),100,1)
% end
% 
% % Close the file
% fclose(fileID);
% 
% disp(['Waypoints written to ', filename]);






%% WAYPOINT SOLVER PLOT
figure
hold on
show(cs);
title("Exhaustive Solver Algorithm")

% [wp,soln]=plan(cp,takeoff);
% hold on

%plot(wptsExh(:,1),wptsExh(:,2),LineWidth=1.5);
plot(wptsMin(:,1),wptsMin(:,2),LineWidth=1.5,LineStyle="--");
plot(takeoff(1),takeoff(2),MarkerSize=25,Marker=".")

hold on
for i = 1:length(ObsticalVertices)
    vertices = ObsticalVertices{i}; % Extract vertices from the cell
    buffverts = BuffObstical{i}.Vertices;
    fill([buffverts(:,1); buffverts(1,1)], [buffverts(:,2); buffverts(1,2)], 'k','FaceAlpha',.4,'EdgeColor','k'); 
    fill([vertices(:,1); vertices(1,1)], [vertices(:,2); vertices(1,2)], 'r','FaceAlpha',.2,'EdgeColor','k')
end

plot(Ix1,Iy1)
plot(Ix2,Iy2)
scatter(crit(1,1),crit(1,2))
scatter(crit(1,3),crit(1,4))

title("Coverage Plan")
xlabel('Easting (m)')
ylabel('Northing (m)')
hold off



figure;
hold on
plot(buffer,'k')
plot(costj,'r')
yline(1-target,'--',color='k')
title("Buffer size")
xlabel('Iteration')
ylabel('Value')
legend('Buffer size (m)','Risk (%)')
hold off