clc
clear
close all
%% Latest Verion of RiskAwarePlanner.m
%to do list
%look at PRM to increase resolution
%add shallow obstical collision and cell generation check (part done)
%fix sigma function using newest dataset and scalar
%prm nodes near obbys
%positble tsp adjust
%swath width calc (done)
%combine my risk obby with inwater obby (may not be issue)
%rotation matrix for dataset waypoints (works for data set, finish wpt out)
%add time to travel enviornment (time min,sec,distance done)

%Style plots for paper 


%% ROI survey

%init
cellnum=15; %cells per interpolated point
shallowcut=[-.1,.2]; %pdf intigration region
target = .95; %target accuracy 
counter = 1; %counter term for while loop
ie=0; %integral error
swathwidthpercent = .1;
ObsticalVertices=[]; %init obverts
riskobby=[];
buffriskobby=[];
OccObstical=[];
occrisk=[];
highriskobby=[];

%% ROI survey input / data location
outline=readtable("TestData\Test_22\Test22_outline_high.txt");
%outline=readtable("TestData\Test_8\test8_outline.txt");

long0 = outline.GPSLongitude;
lat0 = outline.GPSLatitude;
%alt0= outline.GPSElevation;
%alt0= outline.Elevation;
alt0=outline.Altitude;

%DD -> meters (m)
refLat= min(lat0);  % Latitude of Charlotte, NC
refLon = min(long0); % Longitude of Charlotte, NC
refAlt = 0;        % Altitude of Charlotte, NC 


enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');

easting= enu(:,1);
northing= enu(:,2);
alt=enu(:,3);


%% Rotation Matrix

[easting,northing]=rotation(easting,northing);

rotationangle = input('Enter the rotation angle:');
rotationangle = deg2rad(rotationangle);


R = [cos(rotationangle), -sin(rotationangle); sin(rotationangle), cos(rotationangle)];
rotpoint = R*[easting';northing'];

easting=rotpoint(1,:)';
northing=rotpoint(2,:)';

eastingoffset=min(easting);
northingoffset=min(northing);

easting = easting - eastingoffset;
northing = northing - northingoffset;





depth=outline.Depth_m;
%depth=outline.Corr_Depth2;
depth = depth;

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
z=z; % take out 
zcov=zcov;



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
p2=normcdf(shallowcut(1,2),z,zcov); % tranducer depth
r=p2-p1;



%% swath width calc
%avgdepth = mean(outline.Corr_Depth1);



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


avgdepth = mean(depth);

bw=5; %beamwidth 
%beam width spec 5degrees/26degrees
bw=bw/2;
bw= deg2rad(bw);
radius=avgdepth*tan(bw);
radius = sqrt(radius/pi);

areaper = area(polyroi);
swathwidth = (swathwidthpercent * areaper)/(2*radius*sqrt(areaper));
swathwidth = sqrt(areaper)/swathwidth;

if swathwidth < 2
    swathwidth = 2;
end


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
for i=1:obnum
    polygon=polyshape(ObsticalVertices{i});
end


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
    highriskobby{1,i} = rmholes(highriskobby{1,i});
    polyroi = subtract(polyroi,highriskobby{1,i});
   
end
polyroi = rmholes(polyroi);
for i = 1:numel(highriskobby)
    if overlaps(highriskobby{1,i},polyroi) == 1
    riskobby{1,iii}=highriskobby{1,i};
    iii=iii+1;
    end
end


%% filter risk
% for i = 1:size(riskobby,2)
%     if isempty(subtract(riskobby{1,i},polyroi)) == 0
% 
%     else
%         riskobby{1,iii}=highriskobby{1,i};
%         iii=iii+1;
%     end
% end



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
    cs=[];
    


%% Survey Obby Inflation

 % Buffer distance (m)
if numel(obnum)>0
for i=1:obnum
    polygon=polyshape(ObsticalVertices{i});
    BuffObstical{i} = polybuffer(polygon,Buff);
    OccObstical{i}=polybuffer(polygon,Buff-.2);
end
end

if numel(riskobby)> 0
for i=1:numel(riskobby)
    buffriskobby{i}=polybuffer(riskobby{i},Buff);
    occrisk{i}=polybuffer(riskobby{i},Buff-.2);
end
end


for i=1:numel(riskobby)
    


end


newroi = polybuffer(polyroi,-Buff); %% change -2 to buff at some point
occroi = polybuffer(polyroi,-Buff+.2);
roiVertices = [];
roiVertices = newroi.Vertices(:,:);



    cell{1} = newroi;
%% Decomposition
if obnum == 0
    cell{1} = newroi;
else
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

    j=1;
    while j <= numel(cell)

    if isinterior(cell{j},minX,minY)== 1 && isinterior(cell{j},maxX,maxY)== 1 %single cell
    
    celln{1,1} = intersect(cell{j},cutshape1);
    celln{1,2} = intersect(cell{j},cutshape3);
    celln{1,3} = intersect(cell{j},cutshape4);
    celln{1,4} = intersect(cell{j},cutshape2);

    

    cell=[cell(1:j-1),celln,cell(j+1:end)];
    celln=[];
    j=j+4;

    elseif isinterior(cell{j},minX,minY)== 1 && isinterior(cell{j},maxX,maxY)== 0 %left cell only
    
    celln{1,1} = intersect(cell{j},cutshape1);
    celln{1,2} = intersect(cell{j},cutshape3);
    celln{1,3} = intersect(cell{j},cutshape4);

    
    
    cell=[cell(1:j-1),celln,cell(j+1:end)];
    celln=[];
    j=j+3;
    elseif isinterior(cell{j},minX,minY)== 0 && isinterior(cell{j},maxX,maxY)== 1 %right cell only

    celln{1,1} = intersect(cell{j},cutshape3);
    celln{1,2} = intersect(cell{j},cutshape4);
    celln{1,3} = intersect(cell{j},cutshape2);

    
    
    cell=[cell(1:j-1),celln,cell(j+1:end)];
    celln=[];
    j=j+3;
    else %no cells
    j=j+1;
    % cellorder(i,j)=1;
    % cell{end+1}=fakecells{j};

    end
    end

for i = 1:numel(BuffObstical)
    for j = 1:numel(cell)
    cell{j}=subtract(cell{j},BuffObstical{i});
    %fprintf('%d  %d',i,j)
    end
end




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

%roiVertices = cell{1,i*3+1}.Vertices(:,:);
end
end



fakecells = cell;
%cell=[];
cellorder = zeros(numel(riskobby),numel(fakecells));
%% discritize risk points

if numel(buffriskobby) >0
for i = 1:numel(riskobby)

    [minX, idx] = min(buffriskobby{1,i}.Vertices(:,1));
    minY = min(buffriskobby{1,i}.Vertices(idx,2));
    [maxX, idx] = max(buffriskobby{1,i}.Vertices(:,1));
    maxY = min(buffriskobby{1,i}.Vertices(idx,2));

    lineminX = [minX,minX];
    lineminY = [min(roiVertices(:,2))-2,max(roiVertices(:,2))+2];

    linemaxX = [maxX,maxX];
    linemaxY = [min(roiVertices(:,2)),max(roiVertices(:,2))];


    cutshape1 = polyshape([0 0 minX minX],[min(roiVertices(:,2))-2 max(roiVertices(:,2))+2 max(roiVertices(:,2))+2 min(roiVertices(:,2))-2 ]);
    cutshape2 = polyshape([12000 12000 maxX maxX],[min(roiVertices(:,2))-2 max(roiVertices(:,2))+2 max(roiVertices(:,2))+2 min(roiVertices(:,2))-2 ]);
    cutshape3 = polyshape([minX minX maxX maxX],[minY 0 0 maxY]);
    cutshape4 = polyshape([minX minX maxX maxX],[minY 10000 10000 maxY]);
    j=1;
    while j <= numel(cell)

    if isinterior(cell{j},minX,minY)== 1 && isinterior(cell{j},maxX,maxY)== 1 %single cell
    
    celln{1,1} = intersect(cell{j},cutshape1);
    celln{1,2} = intersect(cell{j},cutshape3);
    celln{1,3} = intersect(cell{j},cutshape4);
    celln{1,4} = intersect(cell{j},cutshape2);

    

    cell=[cell(1:j-1),celln,cell(j+1:end)];
    celln=[];
    j=j+4;

    elseif isinterior(cell{j},minX,minY)== 1 && isinterior(cell{j},maxX,maxY)== 0 %left cell only
    
    celln{1,1} = intersect(cell{j},cutshape1);
    celln{1,2} = intersect(cell{j},cutshape3);
    celln{1,3} = intersect(cell{j},cutshape4);

    
    
    cell=[cell(1:j-1),celln,cell(j+1:end)];
    celln=[];
    j=j+3;
    elseif isinterior(cell{j},minX,minY)== 0 && isinterior(cell{j},maxX,maxY)== 1 %right cell only

    celln{1,1} = intersect(cell{j},cutshape3);
    celln{1,2} = intersect(cell{j},cutshape4);
    celln{1,3} = intersect(cell{j},cutshape2);

    
    
    cell=[cell(1:j-1),celln,cell(j+1:end)];
    celln=[];
    j=j+3;
    else %no cells
    j=j+1;
    % cellorder(i,j)=1;
    % cell{end+1}=fakecells{j};

    end
    end
    
end

for i = 1:numel(buffriskobby)
    for j = 1:numel(cell)
    cell{j}=subtract(cell{j},buffriskobby{i});
    %fprintf('%d  %d',i,j)
    end
end
end
%% convert polygons to vertices


BuffObstical = [BuffObstical,buffriskobby];
OccObstical = [OccObstical,occrisk];

if numel(BuffObstical) > 0
    for i= 1:size(cell,2)
    polygons{1,i} = cell{1,i}.Vertices;
    end
else
    polygons{1,1} = newroi.Vertices;
end


%% occapancy map
mapsize = [max(easting),max(northing)];
reso = 10;
map = occupancyMap(mapsize(1), mapsize(2), reso);
[xGrid, yGrid] = meshgrid(1:(1/(reso*2)):mapsize(1), 1:(1/(reso*2)):mapsize(2));
obsMask = inpolygon(xGrid, yGrid, occroi.Vertices(:,1), occroi.Vertices(:,2));
setOccupancy(map, [xGrid(obsMask), yGrid(obsMask)], 0);

if numel(OccObstical) > 0
    for i = 1:numel(OccObstical)  
    obsMask = inpolygon(xGrid, yGrid, OccObstical{1,i}.Vertices(:,1), OccObstical{1,i}.Vertices(:,2));
    setOccupancy(map, [xGrid(obsMask), yGrid(obsMask)], 1);
    end
end

if numel(BuffObstical) < 1
    polygons{1} = polygons{1};
end


[xxx,yyy] = [centroid(cell{1})];
takeoff= [xxx,yyy,0];
landing=[xxx,yyy,0];

%% test feasablility of cpp on each cell, replace with better cells
j=1;
p=0;
while j <= numel(polygons)
fprintf('Cell swathangle checking, current cell :%d, out of %d cells \n',j,numel(polygons))

try 
cs = uavCoverageSpace(Polygons=polygons{j},UseLocalCoordinates=true);
cpMin = uavCoveragePlanner(cs,Solver="MinTraversal");
[wptsMin,solnMin] = plan(cpMin,takeoff,landing);

catch

    warning('cell divided into subcells')

    newshape = polyshape(polygons{j}(:,1),polygons{j}(:,2));
    [xvert,yvert] = centroid (newshape);
    
    cutshape1 = polyshape([0 0 xvert xvert],[min(roiVertices(:,2))-2 max(roiVertices(:,2))+2 max(roiVertices(:,2))+2 min(roiVertices(:,2))-2 ]);
    cutshape2 = polyshape([12000 12000 xvert xvert],[min(roiVertices(:,2))-2 max(roiVertices(:,2))+2 max(roiVertices(:,2))+2 min(roiVertices(:,2))-2 ]);

    celln{1,1} = intersect(newshape,cutshape1);
    celln{1,2} = intersect(newshape,cutshape2);



    newpolygon{1} = celln{1,1}.Vertices;
    newpolygon{2} = celln{1,2}.Vertices;


    if j == 1 && numel(polygons) == 1
    polygons = newpolygon;
    elseif j == 1
    polygons = [newpolygon, polygons(j+1:end)];
    else
    polygons = [polygons(1:j-1),newpolygon, polygons(j+1:end)];
    end
    %j=j+numel(newpolygon);
    newpolygon=[];
    Polygons = [];
    cs=[];
    celln=[];
    j=j-1;
end
j=j+1;
Polygons = [];
cs=[];
end



 %% solver
cs = uavCoverageSpace(Polygons=polygons,UseLocalCoordinates=true);
ReferenceHeight = 0;
cs.UnitWidth = swathwidth;
cs.UnitLength = 1;


% setCoveragePattern(cs,1,SweepAngle=90)
% cp = uavCoveragePlanner(cs,Solver="Exhaustive");
% % 
% % takeoff= [roiVertices(1,:),0];
% % landing = [roiVertices(1,:),0];
[xxx,yyy] = [centroid(cell{1})];
takeoff= [xxx,yyy,0];
landing=[xxx,yyy,0];
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


BuffObstical{1,numel(BuffObstical)+1} = newroi; %idk why this is after decomposition

for ii = 1:1:numel(BuffObstical)   %,obnum+1:-1:1,1:1:obnum+1] end at ~496 

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
    
    prm = mobileRobotPRM(map, 300);
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

if obnum > 0 && numel(BuffObstical) > 0
for i = 1:length(ObsticalVertices)
    vertices = ObsticalVertices{i}; % Extract vertices from the cell
    buffverts = BuffObstical{i}.Vertices;
    fill([buffverts(:,1); buffverts(1,1)], [buffverts(:,2); buffverts(1,2)], 'k','FaceAlpha',.4,'EdgeColor','k'); 
    fill([vertices(:,1); vertices(1,1)], [vertices(:,2); vertices(1,2)], 'r','FaceAlpha',.2,'EdgeColor','k')
end

if numel(buffriskobby) > 0
    hold on
    for i = 1:numel(buffriskobby)
    plot(buffriskobby{1,i},FaceColor='r')
    plot(riskobby{1,i},FaceColor='k')
    end
end
end
xlabel('X (m)')
ylabel('Y (m)')
title({sprintf('Iteration: %.0f, Buffer size: %.3f', counter, Buff), 'Path Plan'});
axis equal






%% Risk Calculations

waypoints=[wptsMin(:,1),wptsMin(:,2)];

if ~isempty(ObsticalVertices) == 1
obverts=vertcat(ObsticalVertices{:});
obverts=[obverts;polyroi.Vertices(:,:)];
else
obverts = [polyroi.Vertices(:,:)];
end

[sig,interwayx,interwayy,inter,ang ] = sigmainterpol(X,Y,z,waypoints);

[collisionChance] = usvriskcalculation(X,Y,z,zcov,inter,interwayx,interwayy,sig,cellnum, shallowcut, ObsticalVertices, polyroi.Vertices);

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
for i = 1:numel(buffriskobby)
    plot(buffriskobby{1,i},FaceColor='r')
    plot(riskobby{1,i},FaceColor='k')
end

%% interplot
colormap(gca, "parula"); 
colorbar; 
set(gca);
clim([0 (1-target)*2]);
xlabel('X (m)');
ylabel('Y (m)');
ylabel(colorbar,'Probability')
title('Plot of Collision Chance');
axis equal

cost = max(collisionChance);



%% compute dis to obby
% mindist=[];
% for i = 1:size(inter,1)
% 
%     diffs = obverts - inter(i, :);
%     distances = sqrt(sum(diffs.^2, 2));
% 
% 
%     [min_dist, idx] = min(distances);
%     closest_vertex(i,:) = obverts(idx, :);
% 
%     mindist(i)=min_dist;
% end
% 
% min_mindist = min(mindist);
% max_mindist = max(mindist);
% grayscale_values = (mindist - min_mindist) / (max_mindist - min_mindist); 



diffs1 = diff(waypoints);
lengths = sqrt(sum(diffs1.^2,2));
totalDist(counter)=sum(lengths);

time(counter) = totalDist(counter)/2; %replace with velo
minutes(counter) = time(counter)/60;




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


% ipx = (mindist < 10) & (collisionChance > 1-target);
% if all(ipx == 0)
% fprintf('Risk may still exists, cannot be solved by obstical buffer')
% PATHRISK = 1;
% end

    % wptsMin(:,:) = [];
    % BuffObstical = [];
    % roiVertices = [];
    % cell=[];
    % polygons=[];
    % newwaypoint=[];


end

%% UTM to DD and sat plot


wptsMin(:,1)=wptsMin(:,1)+eastingoffset;
wptsMin(:,2)=wptsMin(:,2)+northingoffset;


R = [cos(-rotationangle), -sin(-rotationangle); sin(-rotationangle), cos(-rotationangle)];
rotpoint = R*[wptsMin(:,1)';wptsMin(:,2)'];

wptsMin(:,1)=rotpoint(1,:)';
wptsMin(:,2)=rotpoint(2,:)';


WGS1984 = enu2lla([wptsMin(:,1), wptsMin(:,2), wptsMin(:,3)], [refLat, refLon, refAlt], 'ellipsoid');
WGS1984 = WGS1984(1:size(WGS1984,1)-1,:);
waypoint = [WGS1984(:,1),WGS1984(:,2)];

figure;
hold on
gx = geoaxes;
geobasemap(gx,'satellite')
% 
geoplot(gx,(WGS1984(:,1)),WGS1984(:,2),lat0,long0,'r-')
title('Waypoint Plot')
hold off


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


title("Coverage Plan")
xlabel('Easting (m)')
ylabel('Northing (m)')
hold off



figure;
subplot(2,1,1)
hold on
plot(buffer,'k',LineWidth=2)
plot(costj,'r',LineWidth=2)
yline(1-target,'--',color='k',LineWidth=2)
scatter(1:size(buffer,2),buffer,100,'k',Marker="+",LineWidth=2)
scatter(1:size(buffer,2),costj,100,'r',Marker="+",LineWidth=2)
title("Buffer size")
xlabel('Iteration')
ylabel('Value')
legend('Buffer size (m)','Risk','Target')
hold off
subplot(2,1,2)
hold on
plot(minutes,'b',LineWidth=2)
scatter(1:size(minutes,2),minutes,100,'b',Marker="+",LineWidth=2)
title("Estimated Survey Time")
xlabel('Iteration')
ylabel('Value')
legend('Time (min)')
hold off






% % easting = easting + eastingoffset;
% % northing = northing + northingoffset;
% % R = [cos(-rotationangle), -sin(-rotationangle); sin(-rotationangle), cos(-rotationangle)];
% % rotpoint = R*[easting';northing'];
% % easting=rotpoint(1,:)';
% % northing=rotpoint(2,:)';