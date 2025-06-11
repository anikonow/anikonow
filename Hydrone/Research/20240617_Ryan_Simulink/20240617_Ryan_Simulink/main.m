clear; close all; clc;

m = 25; % mass, kg
I = eye(3,3)*m*(0.5^2); % inertia, kg-m^3
b = 20; % linear damping
br = 5; % rotational damping
L = 1; % half-length, meters
w = 0.6; % half-width, meters
Simulation_Time = 10; % seconds
Tmax = 30; % max thrust, Newtons

WGS84 = [0 0 0; 5 5 0; 5 5 0 ; 0 0 0];


out = sim('model',Simulation_Time);












% extract data
angles = out.logsout{1}.Values.Data;
pos = out.logsout{2}.Values.Data;
x = pos(:,1);
y = pos(:,2);
psi = angles(:,3);
TL_by = out.logsout{3}.Values.Data;
TR_by = out.logsout{4}.Values.Data;
TL_bx = out.logsout{5}.Values.Data;
TR_bx = out.logsout{6}.Values.Data;

figure;
plot(x,y)


lp_x = [-L L]; % left pontoon x (as viewed from back)
lp_y = [w w]; % left pontoon y
rp_x = [-L L]; % right pontoon x
rp_y = [-w -w]; %  right pontoon y
mid_x = [-1 1 1 -1]*L/2;
mid_y = [1 1 -1 -1]*w;
 lw = 2;
for i = 1:1:length(x)
    xi = x(i);
    yi = y(i);
    psii = psi(i);
    [lp_xtr, lp_ytr] = rotateVals(lp_x,lp_y,xi,yi,psii);
    [rp_xtr, rp_ytr] = rotateVals(rp_x,rp_y,xi,yi,psii);
    [mid_xtr, mid_ytr] = rotateVals(mid_x,mid_y,xi,yi,psii); 
    [TLx, TLy] = rotateVals( [-TL_by 0]./Tmax-L,[-TL_bx 0]./Tmax-w,xi,yi,psii);
    [TRx, TRy] = rotateVals( [-TR_by 0]./Tmax-L,[-TR_bx 0]./Tmax+w,xi,yi,psii);
    if ( i == 1 ) % first frame        
        figh1 = plot( lp_xtr, lp_ytr, 'k-', 'LineWidth', lw  );
        hold on;
        figh2 = plot( rp_xtr, rp_ytr, 'k-', 'LineWidth', lw  );
        figh3 = fill( mid_xtr, mid_ytr, 'k' );
        figh4 = plot( x(1:i) , y(1:i), 'b-');
        figh5 = plot( TLx, TLy, 'r--','linewidth',2);
        figh6 = plot( TRx, TRy, 'm--','linewidth',2);
        figh7 = scatter(WGS84(:,1),WGS84(:,2));
        % xlim([min(x) max(x)])
        % ylim([min(y) max(y)])
        axis equal
        grid on;
    else
        set(figh1,'XData',lp_xtr,'YData',lp_ytr)
        set(figh2,'XData',rp_xtr,'YData',rp_ytr)
        set(figh3,'XData',mid_xtr,'YData',mid_ytr)
        set(figh4,'XData',x(1:i),'YData',y(1:i))    
        set(figh5,'XData',TLx,'YData',TLy)  
        set(figh6,'XData',TRx,'YData',TRy) 
        axis equal;
        xlim([min(x)-2*L max(x)+2*L])
        grid on;        
        drawnow;
        pause (0.1)
        
    end
end

function [xout,yout] = rotateVals(xvec,yvec,x,y,psi)
R_BtoI = [cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0         0        1];
for i = 1:1:length(xvec)
    vec = [xvec(i); yvec(i); 0];
    vecout = R_BtoI*vec;
    xout(i) = vecout(1);
    yout(i) = vecout(2);
end
xout = xout + x;
yout = yout + y;

end


