%% Line width %%
% calculates line width for complete coverage based on function f(x) 
clc
clear
close all


syms u uu z x z2 r2

%enviorment
xi = linspace(0, 100, 100);


z=.001 *(u+150)^2 +5;
x=((uu-5)/.001)^.5 -150;

z= .5*u+5;
x= 2*(uu - 5);

% % z=-.001 *(u-50)^2 +5;
% % x=((uu-5)/-.001)^.5 +50;

zvalues=double(subs(z,u,xi));


x0=0; %starting x point
bw=2.5; %beamwidth
bw= deg2rad(bw);

data=zeros(1,3);
i = 1;
while x0 < 100
    z1= double(subs(z,u,x0));
    r1= double(z1*tan(bw));   
    
    data(i,1)=x0;
    data(i,2)=z1;
    data(i,3)=r1;

    if i>1
     data(i,5)=data(i,1)-data(i-1,1);
    end

    eq1= (sqrt((subs(x,uu,z1) - subs(x,uu,z2))^2 + (z1 - z2)^2 ))-r1 == r2;
    eq2= z1*tan(bw)+z2*tan(bw)- r1 == r2;
    [Z2,R2]=solve([eq1,eq2],[z2,r2]);
    Z2=double(Z2);

    
    x0=double(subs(x,uu,Z2(2,1)));
   

    i=i+1;
    %fprintf('loop ran with i: %f \n',i)
%break
end

data(:,4)= pi*data(:,3).^2;



%plotting
figure
plot(xi,zvalues, 'Color', 'k', 'LineStyle', '--');
hold
viscircles([data(:,1),data(:,2)],data(:,3), 'Color', 'b', 'LineWidth', 1)
title('Survey Line Width');
xlabel('distance')
ylabel('depth')
set(gca,'YDir','reverse')
hold off

figure
plot(data(:,1),data(:,3),'color','r')
hold on
plot(data(:,1),data(:,4),'color','b')
scatter(2:size(data), data(2:size(data),5))
legend 'Radii' 'Area' 'line width'

title('Size of R vs distance')
xlabel('distance')
ylabel('size')
hold off









