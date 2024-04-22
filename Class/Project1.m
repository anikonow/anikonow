clc
clear

load('racetrack_waypoints.txt')

xref=racetrack_waypoints(:,1);
yref=racetrack_waypoints(:,2);
vref=racetrack_waypoints(:,3);


  %%% determine the distance of the road
distance_on_road = zeros(size(xref));
for i = 2:length(xref)
    dx = xref(i) - xref(i-1);
    dy = yref(i) - yref(i-1);
    distance_on_road(i) = distance_on_road(i-1) + hypot(dx, dy);
end


%%Initial conditions and initial error ep

x=xref(1);
y=yref(1);
v=vref(1);
theta=atan2(y(1),x(1))
states=[x;y;theta;v];
error_prev=0;


%%% Time steps and final time. The final time is not exact but close
%%% enough to the actual required time for compeleting the path. 

ts=.01;
tfinal=110

k=1;
for ll=0:ts:tfinal


    [a(k) error_current(k)]=LongitudinalController(states(:,k),...
                            xref,yref,vref,distance_on_road,error_prev); %%% Longitutidanl control
    delta(k)  = LatController(states(:,k),xref,yref,distance_on_road);   %% Lateral Control
   
    states(:,k+1)=model(states(:,k),a(k),delta(k));  %%%% Dynamics of the car
    
    error_prev=error_current(k);                     %% previous errors
    
    k=k+1;
end


%%% plots

xc=states(1,:);
yc=states(2,:);
thetac=states(3,:);
vc=states(4,:);

plot(xref,yref,'r--',xc,yc,'k','LineWidth', 2);
xlabel('x_c','FontSize',14)
ylabel('y_c','FontSize',14)
axis([min(xc)-.1*abs(min(xc)) max(xc)+.1*abs(max(xc)) ...
    min(yc)-.1*abs(min(yc)) max(yc)+.1*abs(max(yc))])
grid on

hold on
 
% 
% Here is the animation 
 numFrames = length(xc);

for frame=1:100:numFrames
    if frame == 1
        h = plot(xc(frame), yc(frame), 'o', 'LineWidth', 1);
         axis([-300, 200, -800, 100]);
    else
        % Update the plot data
        set(h, 'XData', xc(frame), 'YData', yc(frame));
    end
    pause(.1)
end




function delta  = LatController(states,xref,yref,sref)

ld=16; k=3; L=2;  %%%%% These are some sample values.
% You can choosse different values if you want to. But make sure they are
% the same for both controllers. 

XG=states(1);
YG=states(2);
TG=states(3);
VG=states(4);

dists=sqrt((XG-xref).^2+(YG-yref).^2);

[laterror, i]=min(dists);
SG=sref(i);
SG=SG+ld;

SGG=abs(SG-sref);
[longterro, ii]=min(SGG);
% 
if ii>=length(xref)
    ii=length(xref)-1;
end

%% Lateral Ref Error %%

Kp = 2.3;

dy=yref(i+ld)-YG
dx=xref(i+ld)-XG
lateral_error = atan2(dy,dx)    %;
delta = Kp*lateral_error 


%delta = wrapToPi(delta);
% max_delta = deg2rad(30);
% delta = max(-max_delta, min(max_delta, delta));

%%%%% You need to write your Lateral Controller Here. The output of the
%%%% controller is \delta  the steering angle
end




function [a ec]  = LongitudinalController(states,xref,yref,vref,sref,ep)
ld=10; k=3; L=2;  %%%%% These are some sample values.
% You can choosse different values if you want to 
ts=.01;

XG=states(1);
YG=states(2);
ThetaG=states(3);
VG=states(4);

dists=sqrt((XG-xref).^2+(YG-yref).^2);
[laterror, i]=min(dists);
SG=sref(i);
SG=SG+ld;
SGG=abs(SG-sref);
[longterro, ii]=min(SGG)

if ii>=length(xref)
    ii=length(xref)-1;
end
%% Long PID %%
  % PID parameters
    Kp = 1;
    Ki = 0.1;
    Kd = 0.001;

    % Error
    velocity_error = vref(i) - VG;
    ec = ep + velocity_error * ts;

    % PID control
    a = Kp * velocity_error + Ki * ec + Kd * (velocity_error - ep) / ts;
    a = max(-5, min(15, a));

%%%%%% You need to write your PID Controller Here. There are two outputs
%%%%%% for this controller. The first one is your accelration a and the
%%%%%% second on is the error between the current velocity and the desired
%%%%%% velocity. This error will be collected and used as a error_previous
%%%%%% in the future times steps. 

end



function states=model(states,a,delta)
x=states(1);
y=states(2);
theta=states(3);
v=states(4);

%% Simple kinematic here %%

x = x+v*cos(theta);
y = y+v*sin(theta);
theta = theta + delta;
v=v+a;

states = [x;y;theta;v];
%%%%%%% You need to write your Kinematic Model Here

end

% fileID = fopen('Project1.txt', 'w');
% 
% fprintf(fileID, '%d\n', states);
% 
% fclose(fileID);
