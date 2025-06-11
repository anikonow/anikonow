%% GPS and Ping Test %%
clc
clear

% g = GPS;
% g.port_name = "/dev/ttyUSB1";
% g = g.init_port();


p = Ping1D;    % call class constructor
p.port_name = "COM3"; 
%p.port_name = port_name;                    % set port
p.init_port();   % connect to port
device_id = 1;  
%% Prep CSV Files%%

% fid = fopen('gps_data.csv', 'w+'); %make csv 'a' append 'w' writeover
% fprintf(fid, 'Time, Latitude,Longitude,Speed,Status,NumSatellites,Fix\n'); % Header

fed = fopen('ping1d_data.csv', 'w+');
fprintf(fed, 'Time,Distance,Confidence\n'); % Header




%/////////////////////////////////////////////////////////////////////////
% Setup     ->      end

%% Check Speed of Sound
%/////////////////////////////////////////////////////////////////////////
p.gen_getframe( 'general_request', 'speed_of_sound' );      % frame to check speed of sound
p.sendFrame();                                  % send the frame
pause( .1 );                                     % wait for response 

if( p.received_arr{ 1 }.speed_of_sound ~= 343000 ) % is the response what was expected?
    disp( "Incorrect speed of sound for air" );
    %exit();
end

p.received_arr = {};                            % clear the received array 

dist_arr = [];
confidence_arr = [];


%//////////////////////////////////////////////////////////////////////////

for i=1:360
     
          
          fprintf('Loop ran with i = %d\n', i);
          p.gen_getframe( 'general_request', 'distance_simple' ) % generate frame to check depth
          p.sendFrame();                                          % send that frame
          pause(1)
             while numel( p.received_arr ) < 1 % wait until a complete response
                 pause( 0.1 );
             end

           % push the information onto the holding arrays
          dist_arr = [ dist_arr, p.received_arr{ 1 }.distance * 3.28084 / 1000 ]; % m/s
          confidence_arr = [ confidence_arr, 1 - (p.received_arr{ 1 }.confidence / 100 ) ]; % in %

             % Generate confidence bounds
          xconf = [ 1:1:numel(dist_arr), numel(dist_arr):-1:1 ];
          yconf = [ dist_arr+confidence_arr, dist_arr(end:-1:1)-confidence_arr(end:-1:1) ];
          



          datetime_now = datetime('now'); 
          datetime_str = datestr(datetime_now, 'HH:MM:SS'); 
          fprintf(fed, '%s,%f,%f\n', datetime_str, dist_arr(i), confidence_arr(i));
          fprintf('%s,%f,%f\n', datetime_str, dist_arr(i), confidence_arr(i));  

    p.received_arr = {};                    % clear array
    pause( 0.3);                            % wait

 % while( true )
 %            datetime_now = datetime('now'); 
 %            datetime_str = datestr(datetime_now, 'yyyy-mm-dd HH:MM:SS'); 
 %          try
 % 
 %          fprintf( 'Lat: %3.6f, Lat: %3.6f, Speed: %3.2f, Status: %s, Num Sats: %d, Fix: %d \n', g.info.lat, g.info.lon, g.info.speed, g.info.status, g.info.numSats, g.info.fix );
 %          fprintf( fid,'%s,%3.6f, %3.6f, %3.2f, %s, %d, %d \n' ,datestr(datetime_now, 'HH:MM:SS'),g.info.lat, g.info.lon, g.info.speed, g.info.status, g.info.numSats, g.info.fix );
 %          catch
 %         break
 %      end
 % 
 %      pause( 0.1 ); 
 %  break
 % end
  end
%fclose(fid)
fclose(fed)