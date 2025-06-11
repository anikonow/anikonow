clc
clear
close all


survey=readtable("TestData\Data\test26\surveyB1_low.txt");
waypoint = readtable("TestData\Data\test26\pathB.txt");

% geoplot(survey.GPSLatitude,survey.GPSLongitude)

%% convert GPS to (m)

refLat= 35.2271;  % Latitude of Charlotte, NC
refLon = -80.8431; % Longitude of Charlotte, NC
refAlt = 0;        % Altitude of Charlotte, NC 

lat0=survey.GPSLatitude;
long0=survey.GPSLongitude;
alt0=zeros(1,size(survey.GPSLatitude,1))';

enu = lla2enu([lat0, long0, alt0], [refLat, refLon, refAlt], 'ellipsoid');
wnu = lla2enu([waypoint.Var9,waypoint.Var10,zeros(1,size(waypoint.Var2,1))'], [refLat, refLon, refAlt], 'ellipsoid');


minlat=min(enu(:,1));
minlon=min(enu(:,2));

survey.GPSLatitude=enu(:,1)-minlat;
survey.GPSLongitude=enu(:,2)-minlon;

xwaypoint = wnu(7:end,1)-minlat;
ywaypoint = wnu(7:end,2)-minlon;

waypoints = [xwaypoint,ywaypoint];

% waypoints = 1:42;
% 
% % Reshape into pairs
% pairs = reshape(waypoints, 2, []).';
% 
% % Select every other pair
% selectedPairs = pairs(1:2:end, :);
% 
% % Flip the order of every other selected pair
% for i = 1:size(selectedPairs, 1)
%     if mod(i, 2) == 0
%         selectedPairs(i, :) = flip(selectedPairs(i, :));
%     end
% end
% 
% result = reshape(selectedPairs.', 1, []);
% disp(result);
% 
% waypoints=[xwaypoint(result,:),ywaypoint(result,:)];

interwayx = []; % Initialize x-coordinates
interwayy = []; % Initialize y-coordinates
indx=[];

for k = 1:size(waypoints, 1) - 1
    waydis = norm(waypoints(k, :) - waypoints(k + 1, :));
    if waydis <= 15
    
    interx = linspace(waypoints(k, 1), waypoints(k + 1, 1), 3);%(maxdis*3)/(waydis));
    intery = linspace(waypoints(k, 2), waypoints(k + 1, 2), 3);%(maxdis*3)/(waydis));
    
    interwayx = [interwayx, interx(1:end-1)];
    interwayy = [interwayy, intery(1:end-1)];
    indx = [indx,interx(1:end-1)*0+k];
    fprintf("less than 30")
    elseif waydis > 15
    
    p1 = [waypoints(k, 1), waypoints(k, 2)];
    p2= [waypoints(k + 1, 1), waypoints(k + 1, 2)];
    v=p2-p1;
    vunit=v/norm(v);
    splitdis=p1+vunit*20;

    
    interx = [linspace(waypoints(k, 1), splitdis(1, 1), 5),linspace(splitdis(1, 1), waypoints(k + 1, 1), 7)];%(maxdis*3)/(waydis));
    intery = [linspace(waypoints(k, 2), splitdis(1, 2), 5),linspace(splitdis(1, 2), waypoints(k + 1, 2), 7)];%(maxdis*3)/(waydis));
    
    interwayx = [interwayx, interx(1:end-1)];
    interwayy = [interwayy, intery(1:end-1)];
    indx = [indx,interx(1:end-1)*0+k];
    fprintf("greater than 30")
    end
    
end

% Include the last waypoint
interwayx = [interwayx, waypoints(end, 1)];
interwayy = [interwayy, waypoints(end, 2)];
indx=[indx,k];

interwayx = interwayx';
interwayy = interwayy';
indx = indx';


inter = [interwayx,interwayy];



figure;
hold on
scatter(inter(:,1),inter(:,2),"r",marker=".")
plot(waypoints(:,1),waypoints(:,2),'k')
legend('\mu points','waypoint path')
xlabel('X (m)')
ylabel('Y (m)')
title('\mu Locations')
axis equal




figure;
hold on
plot(survey.GPSLatitude(620:4500,:),survey.GPSLongitude(620:4500,:),'k',linewidth=2)
scatter(waypoints(:,1),waypoints(:,2),"+", MarkerEdgeColor="b",LineWidth=2)


for k = 1:size(inter, 1)
   
    mu = inter(k, :);

    if  norm(inter(k,:)-waypoints(indx(k),:)) >= 15
        sigmax = .33;
    else
    
    %solve for angle of turn
    if indx(k) == 1 || indx(k) ==size(waypoints, 1) 
    angle = 90;

    else
    v1=waypoints(indx(k)-1,:)-waypoints(indx(k),:);
    v2=waypoints(indx(k),:)-waypoints(indx(k)+1,:);
    dotp=dot(v1,v2);
    normv1=norm(v1);
    normv2=norm(v2);
    angle=acos(dotp/(normv1*normv2));
    angle=rad2deg(angle);
    end
    Var = (-1.85*10^-5)*(angle).^2 + .0151*(angle) + 1.7;
    Decay= (-3.08*10^-6)*(angle).^2+.0014*(angle);
    Var = (-1.85*10^-5)*(angle).^2 + .0171*(angle) + 1.7;
    Decay= (-3.08*10^-6)*(angle).^2+.0014*(angle);

    sigmax = Var - (Decay)*(norm(inter(k,:)-waypoints(indx(k),:))/2);
    end
    sigma= [sigmax 0; 0 sigmax];

    theta = linspace(0, 2*pi, 300);
    circle = [cos(theta); sin(theta)]; % Unit circle
    ellipse = (sigma * circle)'; % Transform circle to ellipse
    ellipse = bsxfun(@plus, ellipse, mu); % Shift to waypoint position
    
    
    plot(ellipse(:,1), ellipse(:,2), 'r',linewidth=2);

    sig(k)=sigmax;
end
    
    axis equal

legend('GNSS Trace',"Waypoints","Covariance Ellipse",fontsize=12)
xlabel('X (m)',fontsize=14)
ylabel('Y (m)',fontsize=14)
title('Vessel Covariance Ellipse',fontsize=14)


figure;
plot(1:size(sig,2),sig)
xlabel('interplolated point')
ylabel('variance (m) ')


% figure;
% subplot(1,2,1)
% hold on
% scatter(inter(:,1),inter(:,2),"r",marker="+")
% plot(waypoints(:,1),waypoints(:,2),'k')
% legend('\mu points','waypoint path')
% xlabel('X (m)')
% ylabel('Y (m)')
% axis equal
% hold off


