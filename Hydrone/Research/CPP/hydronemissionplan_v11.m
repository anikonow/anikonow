%% hydrone mission plan V1.2
%v1.1 implemented  drawpolygon feature for mission planner w/ mesh grid
%v1.2 implemented C-Space, Colision detection, 
% simple horizontal waypoint points
%
%
%
%
%
%
%

clc
clear
close all

satim=imread("Hecken.JPG");

poscord1=[35304258, 80732018];
poscord2=[35303966, 80732337];

[xlocal, ylocal]= gps_to_feet(poscord1(1,1), poscord1(1,2),poscord2(1,1), poscord2(1,2));
% Mission params

% numDotsPerRow = (poscord1(1,1)-poscord2(1,1)) *.04;
% numDotsPerColumn = (poscord1(1,2)-poscord2(1,2)) *.03; 

rectWidth = size(satim,2);
rectHeight = size(satim,1);

% Create a grid of dots
[xGrid, yGrid] = meshgrid(linspace(0, rectWidth, xlocal), linspace(0, rectHeight, ylocal));

% Reshape the grid into column vectors
x = xGrid(:);
y = yGrid(:);

% Draw a polygon and get its vertices
figure;
plot(x, y, '.');
hold on
imshow(satim);
hold off
title('Draw a polygon for the region of interest');

roi = drawpolygon;
roiVertices = roi.Position;
roiVertices(size(roiVertices,1)+1,1)=roiVertices(1,1);
roiVertices(size(roiVertices,1),2)=roiVertices(1,2);

close

%Draw Obstacles

%Draw Start and Stop Point

%% Identify Events %% 
%
%will change for drawn direction, check interor (sides-2)*180 = poly
%
%save interior events
%
%discritize cells, including drawn obstacles
%
%plot cells via lawn mower
%
%calculate cost of the discritized cells
%
%Choose lowest cost combination of cells

% Create a rectangle using polygon vertices

%Vertices = [5, 5; 15, 5; 15, 15; 5, 15; 5, 5]; %currently 10x10 box inside space
insideRectangle = inpolygon(x, y, roiVertices(:, 1), roiVertices(:, 2));

% Filter dots inside the rectangle
xInside = x(insideRectangle);
yInside = y(insideRectangle);
sortedDots = sortrows([xInside, yInside], [2, 1]);

%% Collision detection
hydroneR= 12;
jj=1;
ii=1;
for i=1:size(sortedDots, 1)
    coltest= sortedDots(i,:);
    coltestx= [coltest(:,1)+hydroneR,coltest(:,2)];
    coltesty= [coltest(:,1),coltest(:,2)+hydroneR];
    coltestxn= [coltest(:,1)-hydroneR,coltest(:,2)];
    coltestyn= [coltest(:,1),coltest(:,2)-hydroneR];

    if inpolygon(coltestx(:,1),coltestx(:,2),roiVertices(:, 1), roiVertices(:, 2)) == 1 && ...
            inpolygon(coltestxn(:,1),coltestxn(:,2),roiVertices(:, 1), roiVertices(:, 2)) == 1&& ...
            inpolygon(coltesty(:,1),coltesty(:,2),roiVertices(:, 1), roiVertices(:, 2)) == 1&& ...
            inpolygon(coltestyn(:,1),coltestyn(:,2),roiVertices(:, 1), roiVertices(:, 2)) == 1
        
        validmesh(jj,:)= sortedDots(i,:);
        jj=jj+1;
    else
        invalidmesh(ii,:)= sortedDots(i,:);
        ii=ii+1;
    end
%fprintf("loop compleded %f\n", i)
end

%Lineverts=zeros(1,2);
%% Create Simple Lawnmower
PathStart=zeros(1,2);
PathEnd= zeros(1,2);

j=1;
PathStart(1,:)=validmesh(1,:);
for i=1:size(validmesh,1)-1
    if norm([(validmesh(i+1,1)-validmesh(i,1)),(validmesh(i+1,2)-validmesh(i,2))],2)>=16
        PathEnd(j,:)=validmesh(i,:);
        PathStart(j+1,:) = validmesh(i+1,:);
        j=j+1;
        %fprintf("loop compleded %f\n", j)
    end
end
PathEnd(j,:)= validmesh(size(validmesh,1),:);
k=1;
for i=1:2:size(PathStart)*2
    path(i,:)= PathStart(k,:);
    path(i+1,:)=PathEnd(k,:);
    k=k+1;
end

%% TSP with 0 cost nodes 
[finalpath,cost] = Gtsp(path,roiVertices);
finalpath = finalpath';
[~, order] = sort(finalpath);


%polybool -> unionionizes cells

% polygons = coverageDecomposition(roiVertices,"IntersectionTol",.1);
% cs = uavCoverageSpace(Polygons=polygons);
% show(cs);



%need to add function where if outside of polygon cost increases
%% Plot
figure;

imshow("Hecken.jpg")
hold on
for j=1:size(PathStart,1)-1
    plot([PathStart(j,1),PathEnd(j,1)],[PathStart(j,2),PathEnd(j,2)],'b', 'LineWidth', 1);
    %fprintf('%f',j);
end
plot(validmesh(:,1),validmesh(:,2),'.','Color','g')
plot(invalidmesh(:,1),invalidmesh(:,2),'.','Color','r')
plot(sortedDots(:, 1), sortedDots(:, 2), 'square','MarkerSize',5,'Color','k');
plot(roiVertices(:, 1), roiVertices(:, 2), 'k', 'LineWidth', 2); % Show the drawpoly
plot(path(finalpath,1),path(finalpath,2),'color','g','LineWidth',3,'LineStyle','-.')

%
set(gca, 'YDir', 'reverse');

xlabel('X-axis');
ylabel('Y-axis');

title('Waypoint Mission Planning');
hold off;






% remake this function eventually
function [x_distance_feet, y_distance_feet] = gps_to_feet(lat1, lon1, lat2, lon2)

    x= lat1 - lat2;
    y= lon1 - lon2;

    x_distance_feet=abs(x)*.25;
    y_distance_feet=abs(y)*.25-.75;

end


