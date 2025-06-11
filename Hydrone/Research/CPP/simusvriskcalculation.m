function [collisionChance ] = usvriskcalculation(X,Y,z,zcov,inter,interwayx,interwayy,sig,cellnum, shallowcut, ObsticalVertices, roiVertices,counter,Buff)
% X Y z zcov- ROI lat lon depth and depth var
% inter, interwayx, interwayy- interpolated points from function
% sig, var for each point
% cellnum, 


figure;
for k = 1:size(inter, 1)

xg=linspace(inter(k,1)-sig(k),inter(k,1)+sig(k),cellnum);
yg=linspace(inter(k,2)-sig(k),inter(k,2)+sig(k),cellnum);
xdif=xg(2)-xg(1);
ydif=yg(2)-yg(1);

[xgrid,ygrid]=meshgrid(xg,yg);

Zq = interp2(X,Y,z,xgrid,ygrid);
Cq = interp2(X,Y,zcov,xgrid,ygrid);

if size(ObsticalVertices,2) > 0
for ii=1:size(ObsticalVertices,2)
verts=ObsticalVertices{ii};
xpoly=verts(:,1);
ypoly=verts(:,2);
[inPoly, onPoly] = inpolygon(xgrid, ygrid, xpoly, ypoly);
mask = inPoly | onPoly;
Zq(mask) = 0;
Cq(mask) = 0;
end
end

xpoly = roiVertices(:,1);
ypoly = roiVertices(:,2);
[inPoly, onPoly] = inpolygon(xgrid, ygrid, xpoly, ypoly);
mask = ~(inPoly | onPoly);
Zq(mask) = 0;
Cq(mask) = 0;


hold on
surf(xgrid,ygrid,Zq)
shading interp;
grid off
view(2)
colorbar;
colormap(gca, "jet");
for i=1:cellnum
    for j=1:cellnum
    
        xi = xgrid(i,j);
        yj = ygrid(i,j);

        zi=[xi;yj];
        sigma=[(sig(k)/3)^2,0;0,(sig(k)/3)^2];

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


title({sprintf('Iteration: %.0f, Buffer size: %.3f', counter, Buff), 'Depth Mesh'},'Fontsize',14);
xlabel('x (m)','Fontsize',12)
ylabel('y (m)','Fontsize',12)
ylabel(colorbar,'Depth (m)','Fontsize',12)


collisionChance(k) = sum(sum(pobs));





end
% % % plot(inter(:,1),inter(:,2),'k')
axis equal
% % 
% % figure;
% % subplot(1,2,1);
% % hold on
% % surf(X, Y, -z);
% % view(2)
% % colormap(gca, 'parula'); 
% % colorbar;
% % xlabel('X (m)');
% % ylabel('Y (m)');
% % title('Contour Plot');
% % 
% % 
% % subplot(2,1,2);
% % scatter(inter(:,1), inter(:,2), 50, collisionChance(:), 'filled','MarkerEdgeColor','r');
% % colormap(gca, gray); 
% % colorbar; 
% % xlabel('X (m)');
% % ylabel('Y (m)');
% % title('Plot of Collision Chance');






end