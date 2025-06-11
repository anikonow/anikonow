%% USV Uncertainty %%

clc
clear
close all


space = [0 0; 0 10; 10 10; 10 0; 0 0];

waypoints = [2 2; 2 8; 4 8; 4 2; 6 2; 6 8; 8 8; 8 2];


xpoly = [ 0 0 10 10 0]; %x-coords of obstaclr
ypoly = [ 0 4 1 0 0]; %y-coords
%ypoly = [10 6 8 10 10];


maxdis = 0;

% Loop through each pair of adjacent points
for k = 1:size(waypoints, 1) - 1
    distance = norm(waypoints(k, :) - waypoints(k + 1, :)); 
    maxdis = max(maxdis, distance); 
end



Resolution = 50;
X = 0:.5:10;
Y = X;
[X, Y] = meshgrid(X);

F = abs(6 * sin(0.1 * X) .* cos(0.1 * Y) - 2 * cos(0.08 * X - 0.1 * Y) *sin(X*.01)- 4.6);

Xq = [X(:), Y(:)]; 
data = F(:); 

gprMdl = fitrgp(Xq, data, 'KernelFunction', 'squaredexponential');

[z,zcov] = predict(gprMdl, Xq);



%gprMdl = fitrgp(Xq,data,'KernelFunction','squaredexponential');
%[z,zcov] = predict(gprMdl,interpolationPoints);



hold on

z = reshape(z, size(X));
zcov = reshape(zcov, size(X));
surf(X,Y,-z)
plot(waypoints(:,1), waypoints(:,2), color='r', LineStyle='--',LineWidth=2)
plot(xpoly(1,:), ypoly(1,:), color='r', LineStyle='-',LineWidth=2)
legend('Depth','Path','obby')
title('USV workspace')
xlabel('x (m)')
ylabel('y (m)')
colorbar;
axis equal
xlim([0,10])
ylim([0,10])
hold off



figure;
hold on
surf(X, Y, zcov); 
plot(waypoints(:,1), waypoints(:,2), color='r', LineStyle='--',LineWidth=2)
colormap('gray'); 
title('USV Bathy Var')
xlabel('x (m)')
ylabel('y (m)')
colorbar; 
view(2); 
axis equal; 
xlim([0,10])
ylim([0,10])


[sig,interwayx,interwayy,inter] = sigmainterpol(waypoints);


% interwayx = []; % Initialize x-coordinates
% interwayy = []; % Initialize y-coordinates
% indx=[];
% 
% for k = 1:size(waypoints, 1) - 1
%     waydis = norm(waypoints(k, :) - waypoints(k + 1, :));
% 
%     interx = linspace(waypoints(k, 1), waypoints(k + 1, 1), 5);%(maxdis*3)/(waydis));
%     intery = linspace(waypoints(k, 2), waypoints(k + 1, 2), 5);%(maxdis*3)/(waydis));
% 
%     interwayx = [interwayx, interx(1:end-1)];
%     interwayy = [interwayy, intery(1:end-1)];
%     indx = [indx,interx(1:end-1)*0+k];
% end
% 
% % Include the last waypoint
% interwayx = [interwayx, waypoints(end, 1)];
% interwayy = [interwayy, waypoints(end, 2)];
% indx=[indx,k];
% 
% interwayx = interwayx';
% interwayy = interwayy';
% indx = indx';
% 
% 
% inter = [interwayx,interwayy];
% figure;
% hold on
% 
% for k = 1:size(inter, 1)
% 
%     mu = inter(k, :);
% 
%     if  norm(inter(k,:)-waypoints(indx(k)+1,:)) > 2
%         sigmax = .1;
%     else
%     sigmax = .5 - (.1-.5)*(norm(inter(k,:)-waypoints(indx(k)+1,:))/2);
%     end
%     sigma= [sigmax 0; 0 sigmax];
% 
%     theta = linspace(0, 2*pi, 300);
%     circle = [cos(theta); sin(theta)]; % Unit circle
%     ellipse = (sigma * circle)'; % Transform circle to ellipse
%     ellipse = bsxfun(@plus, ellipse, mu); % Shift to waypoint position
% 
% 
%     plot(ellipse(:,1), ellipse(:,2), 'r');
% 
%     sig(k)=sigmax;
% end
%     scatter(waypoints(:,1),waypoints(:,2))
%     plot(xpoly,ypoly,'r')
%     axis equal


%% map generation %%

cellnum = 20; %mesh res
shallowcut= [-1,1]; %shallow water cutoff region

figure;
hold on

for k = 1:size(inter, 1)

xg=linspace(interwayx(k)-sig(k),interwayx(k)+sig(k),cellnum);
yg=linspace(interwayy(k)-sig(k),interwayy(k)+sig(k),cellnum);
xdif=xg(2)-xg(1);
ydif=yg(2)-yg(1);

[xgrid,ygrid]=meshgrid(xg,yg);

Zq = interp2(X,Y,z,xgrid,ygrid);
Cq = interp2(X,Y,zcov,xgrid,ygrid);


[inPoly, onPoly] = inpolygon(xgrid, ygrid, xpoly, ypoly);
mask = inPoly | onPoly;
Zq(mask) = 0;
Cq(mask) = 0;

%surf(xgrid,ygrid,Zq)
% view(2)
sigma=[sig(k),0;0,sig(k)];

for i=1:cellnum
    for j=1:cellnum
    
        xi = xgrid(i,j);
        yj = ygrid(i,j);

        zi=[xi;yj];

        

        pp(i,j)=mvnpdf(zi,inter(k,:)',sigma);
        
        cx = [0 0 1 1]*xdif+xi;
        cy = [0 1 1 0]*ydif+yj;

        om(i,j) = 0; % assume no obstacle
            
            for r = 1:1:4
       
            p1=normcdf(shallowcut(1,1), Zq(i,j),Cq(i,j));
            p2=normcdf(shallowcut(1,2), Zq(i,j),Cq(i,j));
            om(i,j) = p2-p1;
            
            end
    end
end

pp = pp/(sum(sum(pp))); %normalize vessel uncertainty
pobs = om.*pp; %shallow water obstacle collision risk


% title('Interpolation Mesh Depth')
% xlabel('x (m)')
% ylabel('y (m)')


title('Interpolation Mesh risk')
xlabel('x (m)')
ylabel('y (m)')
colorbar
surf(xgrid,ygrid,pp)
view(2)


collisionChance(k) = sum(sum(pobs));


end

figure;
subplot(1,2,1);
hold on
surf(X, Y, -z);
plot(waypoints(:,1),waypoints(:,2),'--',color='r',LineWidth=2)
view(2)
colormap(gca, 'parula'); 
colorbar;
xlabel('X (m)');
ylabel('Y (m)');
title('Contour Plot');


subplot(1,2,2);
scatter(inter(:,1), inter(:,2), 50, collisionChance(:), 'filled','MarkerEdgeColor','r');
colormap(gca, gray); 
colorbar; 
xlabel('X (m)');
ylabel('Y (m)');
title('Plot of Collision Chance');



