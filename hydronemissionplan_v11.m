%% hydrone mission plan V1.1
%implemented  drawpolygon feature for mission planner w/ mesh grid
%
clc
clear

satim=imread("Hecken.JPG");
poscord1=[35304258, 80732018];
poscord2=[35303166, 80730337];
% Mission params

% numDotsPerRow = (poscord1(1,1)-poscord2(1,1)) *.04;
% numDotsPerColumn = (poscord1(1,2)-poscord2(1,2)) *.03; 

rectWidth = size(satim,2);
rectHeight = size(satim,1);

% Create a grid of dots
% [xGrid, yGrid] = meshgrid(linspace(0, rectWidth, numDotsPerRow), linspace(0, rectHeight, numDotsPerColumn));

% Reshape the grid into column vectors
% x = xGrid(:);
% y = yGrid(:);


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

% Create a rectangle using polygon vertices
%Vertices = [5, 5; 15, 5; 15, 15; 5, 15; 5, 5]; %currently 10x10 box inside space
insideRectangle = inpolygon(x, y, roiVertices(:, 1), roiVertices(:, 2));

% Filter dots inside the rectangle
xInside = x(insideRectangle);
yInside = y(insideRectangle);
sortedDots = sortrows([xInside, yInside], [2, 1]);

figure;

hold on
plot(sortedDots(:, 1), sortedDots(:, 2), '.');
plot(sortedDots(:, 1), sortedDots(:, 2), '-b'); % Connect dots with a line
plot(roiVertices(:, 1), roiVertices(:, 2), 'k-', 'LineWidth', 2); % Show the drawpoly
%
set(gca, 'YDir', 'reverse');
xlabel('X-axis');
ylabel('Y-axis');
title('Waypoint Mission Planning');



hold off;