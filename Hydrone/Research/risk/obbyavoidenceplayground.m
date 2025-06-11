clc
clear
close all


waypoints=[0 0;10 0];


load exampleMaps.mat
map = binaryOccupancyMap(complexMap,1);

prmSimple = mobileRobotPRM(map,50);
show(prmSimple)


startLocation = [2 1];
endLocation = [12 10];
path = findpath(prmSimple,startLocation,endLocation);
show(prmSimple)


path = findpath(prmComplex, startLocation, endLocation);
show(prmComplex)








plot(waypoints(:,1),waypoints(:,2))