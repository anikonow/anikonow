%% Line width %%
% calculates line width for complete coverage based on function f(y) 
clc
clear
close all


syms u uu z x z2 r2

%enviorment
lon=[0,100]; %horizontal x
lat=[0,100]; %vertical y


yi = linspace(lat(1,1), lat(1,2), 100); 
xi = linspace(lon(1,1),lon(1,2),100);

% z=.001 *(u+150)^2 +5; 
% x=((uu-5)/.001)^.5 -150; 

z= .5*u+5; %depth function profile of y axis
y= 2*(uu - 5); %postion function of depth

% % z=-.001 *(u-50)^2 +5;
% % x=((uu-5)/-.001)^.5 +50;

zvalues=double(subs(z,u,yi));


y0=0; %starting latitude point
bw=15; %beamwidth 
%beam width spec 5degrees/26degrees
bw=bw/2;
bw= deg2rad(bw);

data=zeros(1,3);
i = 1;
while y0 < 100
    z1= double(subs(z,u,y0));
    r1= double(z1*tan(bw));   
    
    data(i,1)=y0;
    data(i,2)=z1;
    data(i,3)=r1;

    if i>1
     data(i,5)=data(i,1)-data(i-1,1);
    end

    eq1= (sqrt((subs(y,uu,z1) - subs(y,uu,z2))^2 + (z1 - z2)^2 ))-r1 == r2;
    eq2= z1*tan(bw)+z2*tan(bw)- r1 == r2;
    [Z2,R2]=solve([eq1,eq2],[z2,r2]);
    Z2=double(Z2);

    
    y0=double(subs(y,uu,Z2(2,1)));
   

    i=i+1;
    %fprintf('loop ran with i: %f \n',i)
%break
end

data(:,4)= pi*data(:,3).^2;

%% create path over enviorment

% path= [lon(1,1),data(:,1)];
% path2=[lon(1,2),data(:,1)];

path= [repmat(lon(1,1),size(data,1),1),data(:,1)];
path2= [repmat(lon(1,2),size(data,1),1),data(:,1)];


%% create enviroment 

[X,Y] = meshgrid(xi,yi);
Z= double(subs(z,u,Y));



%plotting
figure
plot(yi,zvalues, 'Color', 'k', 'LineStyle', '--');
hold on
scatter(data(:,1),data(:,2), 'Color', 'k')
viscircles([data(:,1),data(:,2)],data(:,3), 'Color', 'b', 'LineWidth', 1)
title('Survey Line Width');
xlabel('Longitude')
ylabel('Depth')
set(gca,'YDir','reverse')
axis equal
hold off

% figure
% plot(data(:,1),data(:,3),'color','r')
% hold on
% plot(data(:,1),data(:,4),'color','b')
% scatter(2:size(data), data(2:size(data),5))
% legend 'Radii' 'Area' 'line width'
% 
% title('Size of R vs distance')
% xlabel('distance')
% ylabel('size')
% hold off

figure
hold on
%surf(X,Y,Z)
imagesc(xi,yi,Z);
shading interp;
colorbar;
colormap(jet)
% set(gca, 'ZDir', 'reverse')
% set(gca, 'Ydir', 'reverse')
for ii= 1:size(data,1)
    plot([path(ii,1),path2(ii,1)],[path(ii,2),path2(ii,2)],"color",'k', 'linewidth',.5)    
end
xlabel('Longitude')
ylabel('Latitude')
zlabel('Depth')
axis equal
hold off





