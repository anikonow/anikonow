%% hydrone mission plan V1.1
%implemented  drawpolygon feature for mission planner w/ mesh grid
%
clc
clear

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

%% Draw a polygon and get its vertices
% figure;
% plot(x, y, '.');
% hold on
% imshow(satim);
% hold off
% title('Draw a polygon for the region of interest');

%% Draw obstacles
% 
% 


%% Identify Events %% 
%
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



%roi = drawpolygon;
roiVertices = [448.810572687225	207.577092511013
376.916299559471	187.488986784141
335.682819383260	183.259911894273
296.563876651982	186.431718061674
233.127753303965	192.775330396476
209.867841409692	201.233480176212
191.894273127753	214.977973568282
179.207048458150	244.581497797357
205.638766519824	341.850220264317
203.524229074890	392.599118942731
191.894273127753	429.603524229075
187.665198237885	576.563876651982
204.581497797357	592.422907488987
244.757709251101	581.850220264317
285.991189427313	545.903083700441
313.480176211454	505.726872246696
357.885462555066	465.550660792952
412.863436123348	454.977973568282
486.872246696035	465.550660792952
680.352422907489	541.674008810573
726.872246696035	576.563876651982
852.687224669604	637.885462555066
864.317180616740	633.656387665198
891.806167400881	606.167400881057
901.321585903084	589.251101321586
894.977973568282	576.563876651982
837.885462555066	534.273127753304
855.859030837005	449.691629955947
849.515418502203	440.176211453745
791.365638766520	442.290748898678
773.392070484582	457.092511013216
744.845814977974	467.665198237885
714.185022026432	460.264317180617
690.925110132159	431.718061674009
687.753303964758	410.572687224670
592.599118942731	297.444933920705
448.810572687225	207.577092511013];

ShapeA= [448.810572687225	207.577092511013
376.916299559471	187.488986784141
335.682819383260	183.259911894273
296.563876651982	186.431718061674
233.127753303965	192.775330396476
209.867841409692	201.233480176212
191.894273127753	214.977973568282
179.207048458150	244.581497797357
205.638766519824	341.850220264317
203.524229074890	392.599118942731
191.894273127753	429.603524229075
412.863436123348	454.977973568282
714.185022026432	460.264317180617
690.925110132159	431.718061674009
687.753303964758	410.572687224670
592.599118942731	297.444933920705
448.810572687225	207.577092511013
448.810572687225	207.577092511013];

ShapeB= [191.894273127753	429.603524229075
187.665198237885	576.563876651982
204.581497797357	592.422907488987
244.757709251101	581.850220264317
285.991189427313	545.903083700441
313.480176211454	505.726872246696
357.885462555066	465.550660792952
412.863436123348	454.977973568282];

ShapeC=[412.863436123348	454.977973568282
486.872246696035	465.550660792952
680.352422907489	541.674008810573
714.185022026432	460.264317180617];

ShapeD=[680.352422907489	541.674008810573
726.872246696035	576.563876651982
852.687224669604	637.885462555066
864.317180616740	633.656387665198
891.806167400881	606.167400881057
901.321585903084	589.251101321586
894.977973568282	576.563876651982
837.885462555066	534.273127753304
855.859030837005	449.691629955947
849.515418502203	440.176211453745
791.365638766520	442.290748898678
773.392070484582	457.092511013216
744.845814977974	467.665198237885
714.185022026432	460.264317180617];

roiVertices(size(roiVertices,1)+1,1)=roiVertices(1,1);
roiVertices(size(roiVertices,1),2)=roiVertices(1,2);

region=[191.894 429.604; 412.863 454.978; 714.185 460.264; 680.352 541.614];
close

% Create a rectangle using polygon vertices
%Vertices = [5, 5; 15, 5; 15, 15; 5, 15; 5, 5]; %currently 10x10 box inside space
insideRectangle = inpolygon(x, y, roiVertices(:, 1), roiVertices(:, 2));

% Filter dots inside the rectangle
xInside = x(insideRectangle);
yInside = y(insideRectangle);
sortedDots = sortrows([xInside, yInside], [2, 1]);

%% Collision detection
hydroneR= 15;
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
a=1;
b=1;
c=1;
d=1;

for i=1:size(validmesh,1)
    if inpolygon(validmesh(i,1), validmesh(i,2), ShapeA(:,1), ShapeA(:,2))
    meshA(a,:)=[validmesh(i,1),validmesh(i,2)];
    a=a+1;

    elseif inpolygon(validmesh(i,1), validmesh(i,2), ShapeB(:,1), ShapeB(:,2))
    meshB(b,:)=[validmesh(i,1),validmesh(i,2)];
    b=b+1;

    elseif inpolygon(validmesh(i,1), validmesh(i,2), ShapeC(:,1), ShapeC(:,2))
    meshC(c,:)=[validmesh(i,1),validmesh(i,2)];
    c=c+1;

    elseif inpolygon(validmesh(i,1), validmesh(i,2), ShapeD(:,1), ShapeD(:,2))
    meshD(d,:)=[validmesh(i,1),validmesh(i,2)];
    d=d+1;
    end
end
    
figure;

imshow("Hecken.jpg")
hold on
% plot(validmesh(:,1),validmesh(:,2),'.')
% plot(invalidmesh(:,1),invalidmesh(:,2),'.')
% plot(sortedDots(:, 1), sortedDots(:, 2), 'square');
plot(meshA(:,1),meshA(:,2))
plot(meshB(:,1),meshB(:,2))
plot(meshC(:,1),meshC(:,2))
plot(meshD(:,1),meshD(:,2))


plot(region(:,1), region(:,2),'-','LineWidth', 3)
plot(roiVertices(:, 1), roiVertices(:, 2), 'g', 'LineWidth', 2); % Show the drawpoly
%
set(gca, 'YDir', 'reverse');

xlabel('X-axis');
ylabel('Y-axis');

title('Waypoint Mission Planning');
hold off;







function [x_distance_feet, y_distance_feet] = gps_to_feet(lat1, lon1, lat2, lon2)

    x= lat1 - lat2;
    y= lon1 - lon2;

    x_distance_feet=abs(x)*.25;
    y_distance_feet=abs(y)*.25-.75;

end