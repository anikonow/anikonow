%% GPS Class for parsing output of EMLID M2 Reach
%
%   This class asynchonously receives the output of the GPS receiver
%   through the specified serial port. NMEA strings are parsed and 
%   relevant information is placed in the 'info' structure within the
%   class. The following is an example of how to implement the class
%   to begin receiving GPS data:
%   
%   %% test GPS class
%   clc; clear all; close all
%
%   g = GPS;
%   g.port_name = "/dev/ttyUSB0";
%   g = g.init_port();
%
%   while( true )
%       try
%           fprintf( 'Lat: %3.6f, Lat: %3.6f, Speed: %3.2f, Status: %s, Num Sats: %d, Fix: %d \n', g.info.lat, g.info.lon, g.info.speed, g.info.status, g.info.numSats, g.info.fix );
%       catch
%       end
%       pause( 0.1 ); 
%   end



classdef GPSEMLID < handle
    %% Properties Definition -> Serial Communication
    %////////////////////////////////////////////////////////////////////
    properties
       port = [];
       port_name = [];
       baud = 57600;
       buf = {};
       info = [];
       lastStr = "";
       last_update = [];
       calibration = [];
       cal_file = 'Data/gps_cal.mat';
    end
    %////////////////////////////////////////////////////////////////////
    % Properties Definition -> end
    
    %% Events Definition    -> Asynchronous Updates
    %////////////////////////////////////////////////////////////////////
    %https://www.mathworks.com/help/matlab/events-sending-and-responding-to-messages.html
    events
       UpdateInfo               % event used to return gps struct back to class
    end
    %////////////////////////////////////////////////////////////////////
    % Events Definition    -> end
    
    %% Constructor          -> GPS()
    %////////////////////////////////////////////////////////////////////
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: GPS()
        %
        % Constructor. Adds listeners which call function which update
        % buffers when packets are received.
        %
        % Input     None
        %
        % Output    None
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function o = GPSEMLID
           addlistener( o, 'UpdateInfo'  , @o.diffInfo );
           
           o.info = GPS.infoStruct_init();
           
           if( exist( o.cal_file, 'file' ) > 0 )
%                 o.loadCalibration( o.cal_file );
           else
                disp( "Calibration should be performed prior to use" ); % add information about how to perform calibration
           end
           
           o.last_update = posixtime( datetime() );
        end
        
        function exportCalibration( o, file_name )
            GPS_CAL = o.calibration;
            save( file_name, 'GPS_CAL' );
        end
        
        function loadCalibration( o, mat_file )
            load( mat_file );
            o.calibration = GPS_CAL;
        end
        
        function o = calibrateGPS( o, reference, duration, save_cal )
            format long
            ultimate_fail_time = duration*2;
            t = GPS.calStruct_init();
            t.last_cal = datetime();
            nSats = [];
            utmx_err = [];
            utmy_err = [];
            lat_err = [];
            lon_err = [];
            i = 1;
            tStart = tic;
            while( i < duration && toc( tStart ) < ultimate_fail_time )
                if( isempty( o.info.lat ) == 0 && isempty( reference.info.gps.lat ) == 0 && isempty( o.info.lon ) == 0 && isempty( reference.info.gps.lon ) == 0 )
                    if( ( o.info.lat ~= -1 ) && ( reference.info.gps.lat ~= -1 ) && ( o.info.lon ~= -1 ) && ( reference.info.gps.lon ~= -1 ) )
                        if( isempty( o.info.numSats ) == 0 && isempty( reference.info.gps_fix_status.num_svs ) == 0 )
                            try
                                [ x1, y1, z1 ] = deg2utm( o.info.lat, o.info.lon );
                                [ x2, y2, z2 ] = deg2utm( reference.info.gps.lat, reference.info.gps.lon );
                                if( z1 ~= z2 ), ccontinue; end
                                utmx_err = [ utmx_err; double( x1-x2 ) ];
                                utmy_err = [ utmy_err; double( y1-y2 ) ];
                                lat_err = [ lat_err; o.info.lat-reference.info.gps.lat ];
                                lon_err = [ lon_err; o.info.lon-reference.info.gps.lon ];
                                nSats = [ nSats; [ o.info.numSats, reference.info.gps_fix_status.num_svs ] ];
                                i = i + 1;
                            catch ME
                                disp( ME );
                            end
                        end
                    end 
                end
                pause( 1 );
            end
            t.utmx_offset = rmoutliers( utmx_err, 'percentiles', [ 10, 90 ] );
            t.utmy_offset = rmoutliers( utmy_err, 'percentiles', [ 10, 90 ] );
            t.utmx_offset = min( t.utmx_offset );
            t.utmy_offset = min( t.utmy_offset );
            t.lat_offset = rmoutliers( lat_err, 'percentiles', [ 10 90 ] );
            t.lon_offset = rmoutliers( lon_err, 'percentiles', [ 10 90 ] );
            t.lat_offset = min( t.lat_offset );
            t.lon_offset = min( t.lon_offset );
            t.lon_cal_vals = lon_err;
            t.lat_cal_vals = lat_err;
            
            t.num_sats = [ [ min( nSats( :, 1 ) ), max( nSats( :, 1 ) ) ], ...
                           [ min( nSats( :, 2 ) ), max( nSats( :, 2 ) ) ] ];
            t.cal_duration = numel( utmy_err );
            t.utmx_cal_vals = utmx_err;
            t.utmy_cal_vals = utmy_err;
            o.calibration = t;
            
            if( save_cal )
                o.exportCalibration( o.cal_file );
            end
            return;
        end
    end
    %////////////////////////////////////////////////////////////////////
    % Constructor          -> end
    
    %%  Information Struct Constructor
    %////////////////////////////////////////////////////////////////////
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Usage: infoStruct_init()
    %
    % Function to simulate typedef struct in c++. Call this function
    % to return a zero/null/empty initialized structure with the information
    % which can be stored about the NMEA strings from the GPS. This
    % function is used to create repeatable structure definitions which
    % seems to be functionality which MATLAB is lacking.
    %
    % Input     None
    %
    % Output    o :         Structure used to house GPS
    %                           parsed data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods ( Static )
        function [ o ] = infoStruct_init()
           o = struct( 'utc', [], 'status', [], 'fix', [], ...
            'lat', [], 'lon', [], 'utm_x', [], 'utm_y', [], 'utm_zone', [], ...
                'speed', [], 'direction', [], 'numSats', [], 'selection_mode', [], ...
                    'mode', [], 'satelites', zeros( 12, 1 ), 'hdop', [], 'pdop', [], 'vdop', [] );
        end
        
        function [ o ] = calStruct_init()
            o = struct( 'last_cal', [], 'num_sats', [], 'cal_duration', [], ...
                'utmx_offset', [], 'utmy_offset', [], 'utmx_cal_vals', [], 'utmy_cal_vals', [], ...
                    'lat_offset', [], 'lon_offset', [], 'lat_cal_vals', [], 'lon_cal_vals', [] );
        end
    end
    %////////////////////////////////////////////////////////////////////
    
    %% Serial Communication Methods
    %////////////////////////////////////////////////////////////////////
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: init_port( {class} ) or {class}.init_port()
        %
        % Use the predefined port name to create a serial port with a callback
        % in the following way:
        %   |-> serialport :                               Matlab Function to create serial port conn.
        %   |-> {class}.port.UserData = @o.notify          Used to pass data back from the callback
        %   |-> configureCallback                          Establish a function call on each packet receipt
        %
        % Input     o :                 Base Class
        %
        % Output    o :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ o ] = init_port( o )
            o.port = serialport( o.port_name, o.baud );        % Establish Serial Port w/ BAUD
            o.port.UserData = @o.notify;                       % Pass notify, see https://www.mathworks.com/help/matlab/ref/handle.notify.html
            configureCallback( o.port, "terminator", @o.cb );  % Set callback on single basic callback function
                                                               %   which handles routing and event notifications
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: diffInfo( {class}, N/A, {ToggleDataEvent struct} ) or {class}.diffInfo( N/A, {ToggleDataEvent struct} )
        %
        % Integrated the most recent data received from the GPS with the
        % the current data. Data which was not received in the most
        % recent packet is reject with this process. Only information
        % which is most recently received accurately will be updated
        % in the 'info' struct within the class.
        %
        % Input     o :                 Base Class
        %           h :          Unused (Callback pass thru)
        %           t :    Structure used for returning callback data (ToggleEventData.m)
        %                   https://www.mathworks.com/help/matlab/matlab_oop/events-and-listeners-syntax-and-techniques.html
        %
        % Output    o :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function o = diffInfo( o, h, t )
            if( isstruct( t.NewState ) == 0 ) 
                return; 
            end
            % log the time of a new update
            if( isfield( t.NewState, 'type' ) )
                if( t.NewState.type == "GNRMC" ) %%changed from "GPRMC to GNRMC
                    o.last_update = posixtime( datetime() );
                end
                t.NewState = rmfield( t.NewState, 'type' );
            end
            replacement = t.NewState;
            fn = fieldnames( o.info );
            for i = 1:numel( fn )
                if( isempty( replacement.(fn{i}) ) == 0 )
                    if( isnumeric( replacement.(fn{i}) ) == 1 )
                       if( replacement.(fn{i}) ~= 0 )
                          o.info.(fn{i}) = replacement.(fn{i}); 
                       end
                    else
                        o.info.(fn{i}) = replacement.(fn{i});
                    end
                        
                end    
            end
        end
    end
    % Serial Communication Methods -> end
    
    %% Parsing Methods              -> GPGGA, GPGSA, GPGSV, GPRMC, GPVTG
    %% New Parsing Methods -> GNGGA, G(ABP)GSA, G(BP)GSV,GNRMC, GNVTG
    % Information for parsers found here:
    % http://freenmea.net/docs
    % http://aprs.gids.nl/nmea/
    %////////////////////////////////////////////////////////////////////
    methods ( Static )
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: parseGPGGA( {class}, NMEA_STRING) or {class}.parseGPGGA( NMEA_STRING )
        %
        % Parse data within NMEA string $GPGGA  
        %
        % Input     o :                 Base Class
        %        data :                NMEA String (GPGGA)
        %
        % Output    o :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ o ] = parseGNGGA( data )
            % data comes in as a pre-split string
            
            %/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            %        1         2      3 4        5 6 7  8   9   10 |  12 13  14   15
            %        |         |      | |        | | |  |   |   |  |  |  |   |    |
            %$--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
            % 1) Time (UTC)
            % 2) Latitude
            % 3) N or S (North or South)
            % 4) Longitude
            % 5) E or W (East or West)
            % 6) GPS Quality Indicator,
            %   0 - fix not available,
            %   1 - GPS fix,
            %   2 - Differential GPS fix
            % 7) Number of satellites in view, 00 - 12
            % 8) Horizontal Dilution of precision
            % 9) Antenna Altitude above/below mean-sea-level (geoid)
            %10) Units of antenna altitude, meters
            %11) Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
            %12) Units of geoidal separation, meters
            %13) Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
            %14) Differential reference station ID, 0000-1023
            %15) Checksum
            %/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            % create temp struct for gps data
            s = GPS.infoStruct_init();
            
            % ensure that the correct number of elements are found, otherwise,
            % it is likely a corrupted/incomplete packet
            if( numel( data ) ~= 15+1 ) 
                o = s;
                return; 
            end
            
            % proceed by adding the derived data to the class structure
            s.utc = data( 2 );
            if( data( 4 ) == 'S' )
                try
%                     s.lat = -1 * str2double( data( 3 ) ) / 100.000;
                    s.lat = -1*GPS.dms2dd( GPS.convert2dms( data( 3 ) ) );
                catch
                    % Error, do not overwrite previous lat value
                end 
            else
                try
%                    s.lat = str2double( data( 3 ) ) / 100.000; 
                    s.lat = GPS.dms2dd( GPS.convert2dms( data( 3 ) ) );
                catch   
                end
            end
            if( data( 6 ) == 'W' )
               try
%                    s.lon = -1 * str2double( data( 5 ) ) / 100.000; 
                   s.lon = -1*GPS.dms2dd( GPS.convert2dms( data( 5 ) ) );
               catch
               end
            else
                try
%                     s.lon = str2double( data( 5 ) ) / 100.000;
                    s.lon = GPS.dms2dd( GPS.convert2dms( data( 5 ) ) );
                catch 
                end
            end
            
            if( isempty( s.lat ) == 0 && isempty( s.lon ) == 0 )
                [ s.utm_x, s.utm_y, s.utm_zone ] = deg2utm( s.lat, s.lon );
            end
            
            try
                s.fix = str2double( data( 7 ) );
            catch
                
            end
            try
               s.numSats = str2double( data( 8 ) ); 
            catch
                
            end
            
            o = s;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: parseGPGSA( {class}, NMEA_STRING) or {class}.parseGPGSA( NMEA_STRING )
        %
        % Parse data within NMEA string $GPGSA  
        %
        % Input     o :                 Base Class
        %        data :                NMEA String (GPGSA)
        %
        % Output    o :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ o ] = parseGPGSA( data )
            % data comes in as a pre-split string
            
            %////////////////////////////////////////////////////////////////
            % .      1 2 3                          14   15 16 17 18
            %        | | |                           |   |   | | |
            % $--GSA,a,a,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x.x,x.x,x.x*hh
            % 1) Selection mode
            % 2) Mode
            % 3) ID of 1st satellite used for fix
            % 4) ID of 2nd satellite used for fix
            % ...
            %14) ID of 12th satellite used for fix
            %15) PDOP in meters
            %16) HDOP in meters
            %17) VDOP in meters
            %18) Checksum
            %////////////////////////////////////////////////////////////////
            
            % create temp struct for gps data
            s = GPS.infoStruct_init();
            
            % ensure that the correct number of elements are found, otherwise,
            % it is likely a corrupted/incomplete packet (+1 for header packet GPGGA)
            if( numel( data ) ~= 18+1 ) 
                o = s;
                return; 
            end
            
            if( data( 2 ) == 'A' )
                s.selection_mode = "Automatic";
            elseif( data( 2 ) == 'M' )
                s.selection_mode = "Manual";
            end
            
            if( data( 3 ) == '1' )
                s.mode = "Fix not available";
            elseif( data( 3 ) == '2' )
                s.mode = "2D";
            elseif( data( 3 ) == '3' )
                s.mode = "3D";
            end
                
            for i = 1:12
               try
                    s.satelites( i ) = str2double( data( i + 3 ) );
               catch
               end
            end
            
            try
                s.hdop = str2double( data( 16 ) );
            catch
            end
            
            try
                s.pdop = str2double( data( 17 ) );
            catch
            end
            
            try
                s.vdop = str2double( data( 18 ) );
            catch
            end
            
            o = s;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: parseGPGSV( {class}, NMEA_STRING) or {class}.parseGPGSV( NMEA_STRING )
        %
        % Parse data within NMEA string $GPGSV  
        %
        % Input     o :                 Base Class
        %        data :                NMEA String (GPGSV)
        %
        % Output    o :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ o ] = parseGPGSV( data ) %%kept might add GBGSV
            % no current method for parsing GPGSV
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: parseGPRMC( {class}, NMEA_STRING) or {class}.parseGPRMC( NMEA_STRING )
        %
        % Parse data within NMEA string $GPRMC  
        %
        % Input     o :                 Base Class
        %        data :                NMEA String (GPRMC)
        %
        % Output    o :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ o ] = parseGNRMC( data )
            % data comes in as a pre-split string

            %////////////////////////////////////////////////////////////////
            %.      1         2 3       4  5       6 7   8   9   10  11 12
            %       |         | |       |  |       | |   |   |    |   | |
            %$--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh
            % 1) Time (UTC)
            % 2) Status, V = Navigation receiver warning
            % 3) Latitude
            % 4) N or S
            % 5) Longitude
            % 6) E or W
            % 7) Speed over ground, knots
            % 8) Track made good, degrees true
            % 9) Date, ddmmyy
            %10) Magnetic Variation, degrees
            %11) E or W
            %12) Checksum
            %////////////////////////////////////////////////////////////////
            
            % create temp struct for gps data
            s = GPS.infoStruct_init();
            
            % ensure that the correct number of elements are found, otherwise,
            % it is likely a corrupted/incomplete packet (+1 for header packet GPGGA)
            if( numel( data ) ~= 13+1 ) 
                o = s;
                return; 
            end
            
            % assign members
            s.utc = data( 2 );
            s.status = data( 3 );
            if( data( 5 ) == 'S' )
                try
%                     s.lat = -1 * str2double( data( 4 ) ) / 100.0000000;
                    s.lat = -1*GPS.dms2dd( GPS.convert2dms( data( 4 ) ) );
                catch
                end
            else
                try
%                     s.lat = str2double( data( 4 ) ) / double( 100.0000000 );
                    s.lat = GPS.dms2dd( GPS.convert2dms( data( 4 ) ) );
                catch
                end
            end
            if( data( 7 ) == 'W' )
                try
%                     s.lon = -1*str2double( data( 6 ) ) / 100.000;
                    s.lon = -1*GPS.dms2dd( GPS.convert2dms( data( 6 ) ) );
                catch
                end
            else
                try
%                     s.lon = str2double( data( 6 ) ) / 100.000;
                    s.lon = GPS.dms2dd( GPS.convert2dms( data( 6 ) ) );
                catch
                end
            end
            
            if( isempty( s.lat ) == 0 && isempty( s.lon ) == 0 )
                [ s.utm_x, s.utm_y, s.utm_zone ] = deg2utm( s.lat, s.lon );
            end
            
            try
                s.speed = str2double( data( 8 ) );
            catch
            end
            o = s;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: parseGPVTG( {class}, NMEA_STRING) or {class}.parseGPVTG( NMEA_STRING )
        %
        % Parse data within NMEA string $GPVTG  
        %
        % Input     o :                 Base Class
        %        data :                NMEA String (GPVTG)
        %
        % Output    o :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ o ] = parseGNVTG( data ) %%Kept
            
            
        end
        
        
        function [ o ] = dms2dd( dms )
            o = -1;
            if( isstruct( dms ) == 0 ), return; end
            try
                o = dms.degrees + dms.minutes / 60.00 + dms.seconds / 3600.00;
                return;
            catch
            end
            return;
        end
        
        function [ o ] = convert2dms( gps_raw )
            o = [];
            try
                g = sprintf( '%5.4f', gps_raw );
            catch
            end
            if( strlength( g ) ~= 9 ), return; end
            
            try
                o.degrees = double( string( g(1:2) ) );
                o.minutes = double( string( g(3:end) ) );
                o.seconds = 0;
            catch
            end
            return;
        end
        
    end
    %////////////////////////////////////////////////////////////////////
    % Parsing Methods              -> end
    
    %% Callback Definition          -> Called on each packet
    %////////////////////////////////////////////////////////////////////
    methods ( Static ) % Static required when class input is not specifically required
        % https://www.mathworks.com/help/matlab/matlab_oop/static-methods.html
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: cb( src, ~ )
        %
        % Function to be called when packet is received  
        %
        % Input     src :         No User Intervention Required
        %           ~   :         Unneeded input
        %
        % Output    None
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function cb( src, ~ )
            data = readline( src );             % read entire received line
            % handle
            try
                substr = split( data, [ ",", "*" ] );
            catch e
                disp( e );
                return
            end
            
            if( numel( substr ) < 1 ) return; end
            switch( substr( 1 ) )
                case { '$GNRMC', 'GNRMC' } %%changed groop GPRMC to GNRMC
                    lastStr = "$GNRMC";
                    %disp( lastStr );
                    t = GPS.parseGPRMC( substr );
                    t.type = "GNRMC";
                    src.UserData( 'UpdateInfo', ToggleEventData( t ) );
                case { '$GNGGA', 'GNGGA' } %%changed group GPGGA GNGGA
                    lastStr = "$GNGGA";
                    %disp( lastStr );
                    t = GPS.parseGPGGA( substr );
                    t.type = "GNGGA";
                    src.UserData( 'UpdateInfo', ToggleEventData( t ) );
                case { '$GPGSA', 'GPGSA' }
                    lastStr = "$GPGSA";
                    t = GPS.parseGPGSA( substr );
                    t.type = "GPGSA";
                    src.UserData( 'UpdateInfo', ToggleEventData( t ) );
                case { '$GPGSV', 'GPGSV' }
                    lastStr = "$GPGSV";
                    %disp( substr );
                case { '$GNVTG' ,'GNVTG' } %%changed group GPVTG to GNVTG
                    lastStr = "$GNVTG";
                    %disp( lastStr );
                otherwise
                    lastStr = "$GPNNN";
                    %disp( lastStr );
            end
        end
        
    end
    % Callback Definition          -> end
    
    
end
% GPS class -> end



    