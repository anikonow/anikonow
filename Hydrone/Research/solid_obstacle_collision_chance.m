clear; close all; clc;

% toy example of chance of collision with a solid obstacle
L = 10; % dimension of a square environment
xpoly = [ 6 6 9 8 6]; %x-coords of obstaclr
ypoly = [ 3 2 7 8 3]; %y-coords

% grid over which probability is defined
Nx = 21;
Ny = Nx;
xgrid = linspace(0,L,Nx);
ygrid = linspace(0,L,Ny);
delx = xgrid(2) - xgrid(1);
dely = ygrid(2) - ygrid(1);
[xx,yy] = meshgrid(xgrid,ygrid);

% p.d.f 
mu = [6; 5]; % mean
sigma = [3 1;1 3];

% loop
for i = 1:1:Nx
    for j = 1:1:Ny
        % get the center of each cell
        xi = xx(i,j);
        yj = yy(i,j);
        % arrange as a vector and check prob
        z = [xi; yj]; 
        pp(i,j) = mvnpdf(z,mu,sigma);
        % define a square cell polygon
        cx = [0 0 1 1]*delx+xi;
        cy = [0 1 1 0]*dely+yj;
        % loop through each corner to see if it inside the obstacle
        om(i,j) = 0; % assume no obstacle
        for k = 1:1:4
            if ( inpolygon(cx(k),cy(k),xpoly,ypoly)==1 && om(i,j)==0 )
                om(i,j) = 1; % there is obstacle
            end
        end
        
    end
end
% we have to normalize the probability so it sums to one
pp = pp/(sum(sum(pp)));

% probability map
figure;
set(gcf,'Units','inches')
set(gcf,'Position',[2 2 16 4])
subplot(1,3,1)
surf(xx,yy,pp);
view(2);
hold on;
plot3(xpoly,ypoly,ones(size(xpoly))*max(max(pp))+0.1,'ro-','linewidth',2);
hold on;
colorbar;
xlim([0 L]);
ylim([0 L]);

% occupancy graph
subplot(1,3,2)
surf(xx,yy,om);
view(2);
hold on;
plot3(xpoly,ypoly,ones(size(xpoly))+0.1,'ro-','linewidth',2);
hold on;
colorbar;
xlim([0 L]);
ylim([0 L]);


% probabilities only over obstacle
pobs = om.*pp;

% plot
subplot(1,3,3)
surf(xx,yy,pobs);
view(2);
hold on;
plot3(xpoly,ypoly,ones(size(xpoly))+0.1,'ro-','linewidth',2);
hold on;
colorbar;
xlim([0 L]);
ylim([0 L]);

% chance of collision
collisionChance = sum(sum(pobs));
fprintf('Collision probability is %3.3f percent \n',collisionChance*100)
