% clc
% clear
% close all
% 
% 
% 
% x = 0:.1:20;
% 
% vesselcov=[0,2;10,.5;15,.3;20,.3];
% depth=[0,2;5,3;10,1;15,.5;20,1];
% 
% p = polyfit(vesselcov(:,1), vesselcov(:,2), 3);
% d = polyfit(depth(:,1),depth(:,2),3);
% 
% % Display the polynomial coefficients
% disp('Sigma coefficients (a, b, c):');
% disp(p);
% 
% % Define the function
% y_func = @(x) p(1)*x.^3 + p(2)*x.^2 + p(3)*x+ p(4);
% 
% y_test = y_func(x);
% 
% 
% d_func = @(x) d(1)*x.^3 + d(2)*x.^2 + d(3)*x+ d(4);
% 
% 
% d_test = d_func(x);
% 
% figure;
% hold on
% plot(x,y_test)
% %scatter(vesselcov(:,1), vesselcov(:,2))
% plot(x,d_test)
% %scatter(depth(:,1),depth(:,2))
% 
% k=0;
% index=1;
% while k < 20
% 
%     waypoint(index,1)=k;
% 
% 
%     cov = y_func(k);
%     depth = d_func(k);
% 
%     nextwaypoint = cov*2 + depth/3;
%     next(index,1)=nextwaypoint;
%     cax(index,1)=cov;
% 
%     k=k+nextwaypoint;
%     index=index+1;
% 
% end

for k = 1:size(waypoints, 1) - 1
    cov1=[];
    depthprofile=[];
    s_prof=[];
    way=[];
    interway=[];
    
    if k == 1 || k ==size(waypoints, 1) 
    angle = 90;
    else
    v1 = waypoints(k-1,:) - waypoints(k,:);
    v2 = waypoints(k,:) - waypoints(k+1,:);

    % Compute the cross product (only the z-component is relevant for 2D vectors)
    crossp = v1(1)*v2(2) - v1(2)*v2(1);

    % Compute the dot product
    dotp = dot(v1, v2);

    % Calculate the magnitude of the angle (in radians)
    angle_rad = atan2(crossp, dotp);

    % Convert to degrees
    angle = abs(rad2deg(angle_rad));
    end

    waydis = norm(waypoints(k, :) - waypoints(k + 1, :));
    Var = (-1.85*10^-5)*(angle).^2 + .0151*(angle) + .3;
    Decay= (-3.08*10^-6)*(angle).^2+.0014*(angle);
    
    linex = linspace(waypoints(k, 1), waypoints(k + 1, 1), 10);%(maxdis*3)/(waydis));
    liney = linspace(waypoints(k, 2), waypoints(k + 1, 2), 10);%(maxdis*3)/(waydis));

    depthprofile = interp2(X,Y,z,linex,liney,'linear');

    s_prof = sqrt((linex - linex(1)).^2 + (liney - liney(1)).^2);
    
    %s_prof=s_prof(:);
    
    
    for i=1:size(linex,2)
    cov1(i) = Var - (Decay)*(norm([linex(i),liney(i)]-waypoints(k,:)));
    if cov1(i) < .3
            cov1(i) = .3;
    end
    end
    
    covfun= @(s) interp1(s_prof,cov1,s,'linear','extrap');
    depthfun = @(s) interp1(s_prof,depthprofile,s,'linear','extrap');
    

    n=0;
    index=1;

    p1 = [waypoints(k, 1), waypoints(k, 2)];
    p2= [waypoints(k + 1, 1), waypoints(k + 1, 2)];
    v=p2-p1;
    vunit=v/norm(v);



    while n < waydis
    
        way(index,1)=n;
    
        cov = covfun(n);
        depthv = depthfun(n);
        
        if depthv>3
            depthv=3;
        end
        if cov>.8
            depthv = 0;
        end

        nextwaypoint = cov*2 + depthv; % interp function

        next(index,1)=nextwaypoint;
        cax(index,1)=cov;

        n=n+nextwaypoint;
        index=index+1;

    end

    for j=1:size(way)
    interway(j,:) = waypoints(k,:) + vunit*way(j); 
    end
if k == 1
    inter = interway;
else
inter=[inter;interway];
end

end

figure;
hold on
scatter(inter(:,1),inter(:,2))
plot(waypoints(:,1),waypoints(:,2))
contour(X,Y,Z)
xlabel('X (m)',FontSize=14)
ylabel('Value (m)',FontSize=14)
legend('\mu Points','Path','Contour',FontSize=12)
axis equal
contour(X,Y,z)
colorbar


% figure;
% scatter(1:size(next),next)