function [sig,interwayx,interwayy,inter,ang ] = sigmainterpol(X,Y,z,waypoints)


interwayx = []; % Initialize x-coordinates
interwayy = []; % Initialize y-coordinates
indx=[];


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

    % Compute the cross product
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
        if cov>.4
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
    indx=ones(size(inter))*k;
else
inter=[inter;interway];
indx=[indx;ones(size(interway))*k];
end

end



for k = 1:size(inter, 1)
   
    mu = inter(k, :);

    if  norm(inter(k,:)-waypoints(indx(k),:)) >= 30
        sigmax = .33;
    else
    
    %solve for angle of turn
    if indx(k) == 1 || indx(k) ==size(waypoints, 1) 
    angle = 90;

    else
    v1 = waypoints(indx(k)-1,:) - waypoints(indx(k),:);
    v2 = waypoints(indx(k),:) - waypoints(indx(k)+1,:);

    % Compute the cross product (only the z-component is relevant for 2D vectors)
    crossp = v1(1)*v2(2) - v1(2)*v2(1);

    % Compute the dot product
    dotp = dot(v1, v2);

    % Calculate the magnitude of the angle (in radians)
    angle_rad = atan2(crossp, dotp);

    % Convert to degrees
    angle = abs(rad2deg(angle_rad));

    % Store the angle
    ang(k) = angle;
    end

    Var = (-1.85*10^-5)*(angle).^2 + .0151*(angle) + .3;
    Decay= (-3.08*10^-6)*(angle).^2+.0014*(angle);

    sigmax = Var - (Decay)*(norm(inter(k,:)-waypoints(indx(k),:)));
    if sigmax < .3
    sigmax=.3;
    end
    
    end
    sigma= [sigmax 0; 0 sigmax];
    sig(k)=sigmax;
end

end
