%% 1D_Ping Test Function %%

clc
clear % clear workspace and command window

%% Setup
%/////////////////////////////////////////////////////////////////////////
% port_name = "/dev/ttyUSB0";                 % set port on Linux machine

p = Ping1D;    % call class constructor
p.port_name = "COM3"; 
%p.port_name = port_name;                    % set port
p.init_port();   % connect to port
device_id = 1;  
%/////////////////////////////////////////////////////////////////////////
% Open the CSV file for writing 
fid = fopen('ping1d_data.csv', 'w+');
fprintf(fid, 'Time,Distance,Confidence\n'); % Header


% Setup     ->      end


%% Check Speed of Sound
%/////////////////////////////////////////////////////////////////////////

p.gen_getframe( "general_request", "speed_of_sound" );      % frame to check speed of sound
p.sendFrame();                                  % send the frame
%pause( 1 );                                     % wait for response 

if( p.received_arr{ 1 }.speed_of_sound ~= 343000 ) % is the response what was expected?
    disp( "Incorrect speed of sound for air" );
    %exit();
end
% 
% p.received_arr = {};                            % clear the received array
%/////////////////////////////////////////////////////////////////////////
% Check Speed of Sound          ->      end


%Create holder variables and figure reference
f = figure();
hold on

dist_arr = [];
confidence_arr = [];

%% Main Loop
%/////////////////////////////////////////////////////////////////////////
for i=1:360
    disp('running');
    p.gen_getframe( 'general_request', 'distance_simple' ) % generate frame to check depth
    p.sendFrame();                                          % send that frame

    while numel( p.received_arr ) < 1 % wait until a complete response
        pause( 0.1 );
    end

    % push the information onto the holding arrays
    dist_arr = [ dist_arr, p.received_arr{ 1 }.distance * 3.28084 / 1000 ]; % m/s
    confidence_arr = [ confidence_arr, 1 - (p.received_arr{ 1 }.confidence / 100 ) ]; % in %

    % Generate confidence bounds
    xconf = [ 1:1:numel(dist_arr), numel(dist_arr):-1:1 ];
    yconf = [ dist_arr+confidence_arr, dist_arr(end:-1:1)-confidence_arr(end:-1:1) ];
    pa = fill( xconf, yconf, 'red' );
    pa.FaceColor = [ 1 0.8 0.8 ];
    pa.EdgeColor = 'none';

    % plot

    plot( dist_arr, 'ko-' );
    xlabel('Event')
    ylabel('depth (ft)')
    drawnow();


    datetime_now = datetime('now'); 
    datetime_str = datestr(datetime_now, 'yyyy-mm-dd HH:MM:SS'); 
    fprintf(fid, '%s,%f,%f\n', datetime_str, dist_arr(i), confidence_arr(i));

    p.received_arr = {};                    % clear array
    pause( 0.4 );                           % wait
end
%/////////////////////////////////////////////////////////////////////////
% Main Loop     ->      end




% Close the CSV file
fclose(fid);
hold off