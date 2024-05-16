clc
clear
close all


stopsign_param = [104.7000  129.0000   38.1000  -90.0000];
x_stopsign=stopsign_param(1);
y_stopsign=stopsign_param(2);
z_stopsign=stopsign_param(3);
psi_stopsign=stopsign_param(4);

Waypoints = readmatrix('project3_waypoints.txt');
SL=1;
ParkedCar = readmatrix('parked_vehicle_params.txt');
x_parkedcar=ParkedCar(1);
y_parkedcar=ParkedCar(2);
z_parkedcar=ParkedCar(3);
psi_parkedcar=ParkedCar(4);

Box_x=ParkedCar(5);    %%%%%% Lets use this dimension for the circle-based collision checking algorithem 
Box_y=ParkedCar(6);
Box_z=ParkedCar(7);

ll=1
for tt=0:.1:2*pi;
    xx(ll)=Box_x*cos(tt)+x_parkedcar;
    yy(ll)=Box_x*sin(tt)+y_parkedcar;
    ll=ll+1;
end

plot(xx,yy,'k--')
hold on



plot(Waypoints(:,1),Waypoints(:,2),x_stopsign,y_stopsign,'ro',x_parkedcar,y_parkedcar, 'go')
hold on

lookahead=15;
egovehicle_params=[Waypoints(1,1) Waypoints(1,2), -pi, 10];
x_egoveh=egovehicle_params(1);
y_egoveh=egovehicle_params(2);
psi_egoveh=egovehicle_params(3);
v_egoveh=egovehicle_params(4);


ts=.1;
tfinal=40;

counter=0;
lookahead=10;


states=[x_egoveh;y_egoveh;psi_egoveh;v_egoveh];
Traj(:,1)=states;


statesleadVeh=[ Waypoints(75,1); Waypoints(75,2); -pi/2;1]; rdynobst=3;  %%% lead vehicle


%%%%%%%%%%%  To Do: Refine the control set if needed
delta=[-pi/4 -pi/8 0 pi/8 pi/4 ];   %% steering control set
aveh=0;    % Initial acceleraiton 

Np=5;  %%% prediction horizon


Time=0:ts:tfinal;


for k=1:length(Time)

   statesleadVeh(:,k+1)=model(statesleadVeh(:,k),0, 0);  
   [d2Waypoints index]=min((Waypoints(:,1)-Traj(1,k)-lookahead*cos(Traj(3,k))).^2+(Waypoints(:,2)-Traj(2,k)-lookahead*sin(Traj(3,k))).^2);
   goalstates=[Waypoints(index,1);Waypoints(index,2)];
   xgoal=goalstates(1); 
   ygoal=goalstates(2);

        ll=1;


    for jj=1:length(delta)

        Delta=delta(jj);

        for ii=1:Np
            states(:,ii+1)=model(states(:,ii),aveh, delta(jj));  
            slength(:,ii+1)=sqrt((states(1,ii+1)-states(1,ii))^2+(states(2,ii+1)-states(2,ii))^2);
        end


        x_prop(jj,:)=states(1,:);
        y_prop(jj,:)=states(2,:);
        theta_prop(jj,:)=states(3,:);
        v_prop(jj,:)=states(4,:);
        s_prop(jj,:)=slength(1,:);

        [d2obst(jj), io(jj)]=min(sqrt((x_prop(jj,2:end)-x_parkedcar).^2+(y_prop (jj,2:end)-y_parkedcar).^2));     %%%% distance to Static Obstacle
        [d2goal(jj), ig(jj)]=min(sqrt((x_prop(jj,2:end)-xgoal).^2+(y_prop(jj,2:end)-ygoal).^2));    %%%% distance to Gaol- Following the Lane
        [d2CL(jj), ig(jj)]=min(sqrt((y_prop(jj,2:end)-ygoal).^2)) ;  %%%% distance to centerline
        [d2leadVeh(jj), ilV(jj)]=min(sqrt((x_prop(jj,:)-statesleadVeh(1,k+1)).^2+(y_prop (jj,:)-statesleadVeh(2,k+1)).^2));   %%%% distance to lead vehicle
        [d2Traffic(jj), iT(jj)]=min(sqrt((x_prop(jj,2:end)-x_stopsign).^2+(y_prop (jj,2:end)-y_stopsign).^2));   %%%% distance to trafficlight

        %%%%%  TO DO: Write Penalty Conditions for Colliion Checking, Steering efforts, Following Goals/Centerline of the road $$$$
        
         P2obst(jj)= exp(-d2obst(jj)^2)
% 
         P2steer(jj)= (abs(Delta)/ max(delta(jj)))
% 
         P2goal(jj)= d2goal(jj)^2*100
% 
         P2CL(jj)= d2CL(jj)^2*5

         

        Ptot(jj)=P2obst(jj)+P2steer(jj)+P2goal(jj)+P2CL(jj);
        [min_val_p jopt]=min(Ptot)  %%% index for the best delta
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%% To Do: Write Velocity Generation Code that follows the lead
    %%%%%%% vehicle
   LD=((Traj(1,k) - statesleadVeh(1,k))^2 + ((Traj(2,k)-10)-statesleadVeh(2,k))^2)^.5;
   if LD < 20
        if LD <5
            LD=1
        end
        v_prop(jopt,:)= LD/2;
   else
        v_prop(jopt,:)=10;
   end
    
    %%%%%%% End of Following Lead Vehicles


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%% To Do: Write Behavioral Path for Dealing with the Stop Sign:
    %%%%%%% Consider 3 states: Lane Follwing, deccelration and stop
    %%% There should be 3 transisitons between Lane following to
    %%% Decceleration, Deccelation to Stop, Stop to Lane Following
    
    SS=((Traj(1,k) - stopsign_param(1))^2 + ((Traj(2,k))-stopsign_param(2))^2)^.5;
    if SS < 20
       v_prop(jopt,:)= 8;
    end
    if SS < 8 % Decre
        v_prop(jopt,:)= SS;
    end
    if SS < 1 && SL < 10 %full stop and pause for 10 time steps
        v_prop(jopt,:)= 0;
        SL=SL+1
    end
    %%%%%%% End of Behavioral Planning
 


    next_x=x_prop(jopt,2);
    next_y=y_prop(jopt,2);
    next_theta=(theta_prop(jopt,2));
    next_v=v_prop(jopt,2);


    states=[next_x;next_y;next_theta;next_v]; 
    Traj(:,k+1)=states;
    plot(Traj(1,k+1),Traj(2,k+1),'ro',statesleadVeh(1,k+1),statesleadVeh(2,k+1),'ko');%,xTraffic,yTraffic,'bo');%,xxlead,yylead,'k--');
    hold on


end







function states=model(states,a,delta)
x=states(1);
y=states(2);
theta=states(3);
v=states(4);
dt=.1;lr=1;L=2;
beta=0;%atan2(lr*tan (delta),L) %%%% setting beta=0 will remove the offset.
vnew=v+a*dt;
xnew=x+v*cos(theta)*dt;
ynew=y+v*sin(theta)*dt;
thetanew=theta+v*cos(beta)*tan(delta)/L*dt;
% deltanew=delta+dt*omega;
states=[xnew;ynew;thetanew;vnew];
end