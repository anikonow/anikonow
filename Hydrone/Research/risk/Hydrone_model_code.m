%% Hydrone_model %%

clc
clear
close all

%mass kg
m=25;
%MoI
i=zeros(3);
i(1,1)= .8125;
i(2,2)= 1.882;
i(3,3)= 2.2945;
%Damping Coe
b=1;
%
r=1;
%
L=1.16;
%
w=.7;



% 
% load("waypoints.mat")
% 
% 
% WGS1984(1,:) = [];
% WGS1984 = WGS1984 - WGS1984(1,:);
% WGS1984 = WGS1984(2:size(WGS1984),:);
% WGS1984 = [WGS1984(:,2),WGS1984(:,1),WGS1984(:,3)];
% WGS1984 = WGS1984 ;

WGS1984 = [ 0 0 0; 10 0 0; 10 10 0; 11 20 0; 20 20 0];%; 30 -10 0 ; 30 10 0];

starthead= atan2(WGS1984(1,2),WGS1984(1,1));

xWpt = WGS1984(:,1);
yWpt = WGS1984(:,2);
zWpt = WGS1984(:,3);
figure; plot(xWpt,yWpt,'bo-')

