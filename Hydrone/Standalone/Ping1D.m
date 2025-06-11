%% Class for Asynchronous Parsing of Ping Sonar 1D from Blue Robotics
%  https://bluerobotics.com/store/sensors-sonars-cameras/sonar/ping-sonar-r2-rp/
%
%   The Ping Sonar Altimeter and Echosounder from BlueRobotics is a
%   single beam echosounder that measures distances up to 100 ft (30m)
%   underwater. The Ping Sonar communicates with RS485 and must therefore
%   use a converted board to interface directly with a computer's USB as
%   is done with this class (See https://bluerobotics.com/store/comm-control-power/tether-interface/bluart-r1-rp/).
%
%   This class is a simple implementation of the Ping Protocol (https://docs.bluerobotics.com/ping-protocol/)
%   developed for use with BlueRobotics sensors. The class simply acts
%   as an encapsulator to send and receive information from the Ping Sonar.
%
%   Additional features are available with this device and protocol; however,
%   they are not currently implemented. They include:
%       * Set parameters on Ping Sonar
%       * Control codes to enter bootloader and start/stop continuous mode
%
%   These features will hopefully be implemented soon.
%
%   Example Usage:
%       This example program polls the sonar and reports distance and confidence
%       measurements assuming air as the medium (343000 mm/s). Currently, alternative methods
%       are required to set the speed of sound in the device as that functionality
%       does not exist in this class.
%
%       clc; clear all; close all                   % prepare workspace
%
%       %% Setup
%       %/////////////////////////////////////////////////////////////////////////
%       port_name = "/dev/ttyUSB0";                 % set port on Linux machine
%
%       p = Ping1D;                                 % call class constructor
%       p.port_name = port_name;                    % set port
%       p.init_port();                              % connect to port
%       %/////////////////////////////////////////////////////////////////////////
%       % Setup     ->      end
%
%       %% Check Speed of Sound
%       %/////////////////////////////////////////////////////////////////////////
%       p.gen_getframe( 'general_request', 'speed_of_sound' );      % frame to check speed of sound
%       p.sendFrame();                                  % send the frame
%       pause( 1 );                                     % wait for response 
%       if( p.received_arr{ 1 }.speed_of_sound ~= 343000 ) % is the response what was expected?
%           disp( "Incorrect speed of sound for air" );
%           exit();
%       end
%
%       p.received_arr = {};                            % clear the received array
%       %/////////////////////////////////////////////////////////////////////////
%       % Check Speed of Sound          ->      end
%
%
%       % Create holder variables and figure reference
%       f = figure();
%       hold on
%
%       dist_arr = [];
%       confidence_arr = [];
%
%       %% Main Loop
%       %/////////////////////////////////////////////////////////////////////////
%       while( true )
%           p.gen_getframe( 'general_request', 'distance_simple' ); % generate frame to check depth
%           p.sendFrame();                                          % send that frame
%    
%           while( numel( p.received_arr ) < 1 )                    % wait until a complete response
%               pause( 0.01 );
%           end
%    
%           % push the information onto the holding arrays
%           dist_arr = [ dist_arr, p.received_arr{ 1 }.distance * 3.28084 / 1000 ]; % m/s
%           confidence_arr = [ confidence_arr, 1 - (p.received_arr{ 1 }.confidence / 100 ) ]; % in %
%    
%           % Generate confidence bounds
%           xconf = [ 1:1:numel(dist_arr), numel(dist_arr):-1:1 ];
%           yconf = [ dist_arr+confidence_arr, dist_arr(end:-1:1)-confidence_arr(end:-1:1) ];
%           pa = fill( xconf, yconf, 'red' );
%           pa.FaceColor = [ 1 0.8 0.8 ];
%           pa.EdgeColor = 'none';
%    
%           % plot
%           plot( dist_arr, 'ko-' );
%           drawnow();
%    
%   
%           p.received_arr = {};                    % clear array
%           pause( 0.1 );                           % wait
%       end
%       %/////////////////////////////////////////////////////////////////////////
%       % Main Loop     ->      end
%       hold off
%
%   End of Example
%
%/////////////////////////////////////////////////////////////////////////////////////////////////////


classdef Ping1D < handle
   %% Properties Definition -> Command Definitions
   %////////////////////////////////////////////////////////////////////
    properties
        % Common frames for all ping protocol
        % https://docs.bluerobotics.com/ping-protocol/pingmessage-common/
        common_frame = {
          %  Num | Name             |       First Response      |           Second Response                 
            { 1, 'ack' ,            { 'u16', 'acked_id'         }                               },
            { 2, 'nack',            { 'u16', 'nacked_id'        }, { 'char', 'nack_message' }   },
            { 3, 'ascii_text',      { 'char', 'ascii_message'   }                               },
            { 6, 'general_request', { 'u16', 'requested_id'     }                               },
            { 100, 'set_device_id', { 'u8', 'device_id'         }                               }
        };
    
        % Ping1D specific get frames
        % https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/#1400-continuous_start
        get_frame = {
          % Num  |      Name                        |       First Response                  |       Second Response             |           Third Response                  |       Fourth Response                 |           Fifth Response              |       Sixth Reponse         |     Seventh Response    |
            { 4,    'get_device_information',       { 'u8', 'device_type' },                { 'u8', 'device_revision' },        { 'u8', 'firmware_version_major' },         { 'u8', 'firmware_version_minor' },     { 'u8', 'firmware_version_patch' },     { 'u8', 'reserved' }                                    },
            { 5,    'get_protocol_version',         { 'u8', 'version_major' },              { 'u8', 'version_minor' },          { 'u8', 'version_patch' },                  { 'u8', 'reserved' }                                                                                                                    },
            { 1200, 'firmware_version',             { 'u8', 'device_type' },                { 'u8', 'device_model' },           { 'u16', 'firmware_version_major' },        { 'u16', 'firmware_version_minor' }                                                                                                     },
            { 1201, 'device_id',                    { 'u8', 'device_d' }                                                                                                                                                                                                                                            },
            { 1202, 'voltage_5',                    { 'u16', 'voltage_5' }                                                                                                                                                                                                                                          },
            { 1203, 'speed_of_sound',               { 'u32', 'speed_of_sound' }                                                                                                                                                                                                                                     },
            { 1204, 'range',                        { 'u32', 'scan_start' },                { 'u32', 'scan_length' }                                                                                                                                                                                                },
            { 1205, 'mode_auto',                    { 'u8', 'mode_auto' }                                                                                                                                                                                                                                           },
            { 1206, 'ping_interval',                { 'u16', 'ping_interval' }                                                                                                                                                                                                                                      },
            { 1207, 'gain_setting',                 { 'u32', 'gain_setting' }                                                                                                                                                                                                                                       },
            { 1208, 'transmit_duration',            { 'u16', 'transmit_duration' }                                                                                                                                                                                                                                  },
            { 1210, 'general_info',                 { 'u16', 'firmware_version_major' },    { 'u16', 'firmware_version_minor' }, { 'u16', 'voltage_5' },                    { 'u16', 'ping_interval' },             { 'u8', 'gain_setting' },               { 'u8', 'mode_auto' }                                   },
            { 1211, 'distance_simple',              { 'u32', 'distance' },                  { 'u8', 'confidence' }                                                                                                                                                                                                  },
            { 1212, 'distance',                     { 'u32', 'distance' },                  { 'u16', 'confidence' },             { 'u16', 'transmit_duration' },            { 'u32', 'ping_number' },               { 'u32', 'scan_start' },                { 'u32', 'scan_length' },    { 'u32', 'gain_setting' }  },
            { 1213, 'processor_temperature',        { 'u16', 'processor_temperature' }                                                                                                                                                                                                                              },
            { 1214, 'pcb_temperature',              { 'u16', 'pcb_temperature' }                                                                                                                                                                                                                                    },
            { 1215, 'ping_enable',                  { 'u8', 'ping_enabled' }                                                                                                                                                                                                                                        },
        };
    
        set_frame = {
            { 1000, 'set_device_id',                { 'u8', 'device_id' } },
            { 1001, 'set_range',                    { 'u32', 'scan_start' },                { 'u32', 'scan_length' } },
            { 1002, 'set_speed_of_sound',           { 'u32', 'speed_of_sound' } },
            { 1003, 'set_mode_auto',                { 'u8', 'mode_auto' } },
            { 1004, 'set_ping_interval',            { 'u16', 'ping_interval' } },
            { 1005, 'set_gain_setting',             { 'u8', 'gain_setting' } },
            { 1006, 'set_ping_enable',              { 'u8', 'ping_enabled' } },
            { 1400, 'continuous_start',             { 'u16', 'id' } },
            { 1401, 'continuous_stop',              { 'u16', 'id' } }
        };
        
        frame = [];                         % storage for frame information, reset by resetFrame()
        port = [];                          % storage for serialport function output
        port_name = [];                     % name of the serial port (i.e. /dev/ttyUSB0)
        baud = 115200;                      % standard baud rate for the Ping Sonar is 115200
        buf = [];                           % buffer which is populated on receipt of information
        received_arr = {};                  % storage location of parsed raw information
        
        device_id = 1;                      % device id is 1...not sure why...
        max_received_arr_size = 50;
        received_word_func = @Ping1D.na;
        continuous_streams = [];
    end
    
   %% Events Definition    -> Asynchronous Updates
   %////////////////////////////////////////////////////////////////////
   %https://www.mathworks.com/help/matlab/events-sending-and-responding-to-messages.html  
    events
       UpdateRec 
    end
    %////////////////////////////////////////////////////////////////////
    % Events Definition    -> Asynchronous Updates      ->  end
    
   %% Constructor          -> Ping1D()
   %//////////////////////////////////////////////////////////////////// 
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: Ping1D()
        %
        % Constructor. Adds listeners which call function which update
        % buffers when packets are received. Initializes blank frame.
        %
        % Input     None
        %
        % Output    o   :   Initialized Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function o = Ping1D
            addlistener( o, 'UpdateRec', @o.pushBuf );
            
            o.frame = o.resetFrame();
        end
    end
    %////////////////////////////////////////////////////////////////////
    % Constructor          -> Ping1D()            ->      end
    
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
        function init_port( o )
            o.port = serialport( o.port_name, o.baud );
            o.port.UserData = @o.notify;
            configureCallback( o.port, "byte", 1, @o.cb );
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: pushBuf( {class}, h, t )
        %
        % Add received information onto the buffer. This action is called
        % through the event callback of 'UpdateRec'. The callback funtion
        % on the serialport leads to this action.
        %
        % Input     o :                 Base Class
        %           h :          Event Provided, no user action required
        %           t :             ToggleEventData passback from notify
        %
        % Output    a :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function o = pushBuf( o, h, t )
            o.buf = [ o.buf; t.NewState ];
            %while( true )
                otpt = o.interpretBuf();
             %   if( otpt.type == "EMPTY" ), break; end
            %end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: getFrameLength( {class}, id, search_space ) or {class}.getFrameLength( id, search_space )
        %
        % Identify the expected length of the response from a particular command.
        % This length is in bytes so a u8 is 1, u16 is 2, u32 is 4. 
        % This call can be used to verify that the received data is 
        % what is expected.
        %
        % Input         o :                 Base Class
        %              id :          Identifier of the frame, command number of name string
        %    search_space :             The structure to be searched for key
        %
        % Output    a :             Updated Base Class
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function a = getFrameLength( o, id, search_space )
            a = -1;
            ind = 0;
            % find what type of id is being used, numeric or string
            if( isstring( id ) )
                id = convertStringsToChars( id );
                ind = 2;
            elseif( ischar( id ) )
                ind = 2;
            elseif( isnumeric( id ) )
                ind = 1;
            end
            
            % with the search type, find the index to be searched
            % within the search space
            fnd = 0;
            for i = 1:size( search_space, 1 )
               v = search_space{ i };
               if( ind == 2 )
                   if( strcmp( v{ 2 }, id ) )
                       fnd = i;
                       break;
                   end
               elseif( ind == 1 )
                   if( v{ 1 } == id )
                        fnd = i;
                        break;
                   end
               end
            end
            if( fnd == 0 ) return; end
            
            v = search_space{ fnd };            % pull the search space index into a variable
            
            pLength = 0;
            for i = 3:size( v, 2 )
                % check in each packet for size
                sz = v{ i }{ 1 };
                switch sz
                    case 'u8'
                        pLength = pLength + 1;
                    case 'u16'
                        pLength = pLength + 2;
                    case 'u32'
                        pLength = pLength + 4;
                    otherwise
                        % char handled later
                end
            end
            
            a = pLength;
        end
        
        function gen_getframe( o, msg_type, request )
            o.frame = o.resetFrame();
            % find msg_type
            % search common frame
            fnd = 0;
            for i = 1:size( o.common_frame, 1 )
                v = o.common_frame{ i };
                if( strcmp( v{ 2 }, msg_type ) )
                    fnd = i;
                    break;
                end
            end
            if( fnd == 0 ), return; end
            
            % set message type in frame
            v = dec2hex( o.common_frame{ fnd }{ 1 }, 4 );
            o.frame.message_id = { v( 3:4 ), v( 1:2 ) }; 
            
            % set payload length
            %v = dec2hex( o.getFrameLength( request, o.get_frame ), 4 );
            v = dec2hex( 2, 4 );
            o.frame.payload_length = { v( 3:4 ), v( 1:2 ) };
            
            % get request_id
            fnd = 0;
            for i = 1:size( o.get_frame, 1 )
                v = o.get_frame{ i };
                if( strcmp( v{ 2 }, request ) )
                    fnd = i;
                    break;
                end
            end
            if( fnd == 0 ) return; end
           
            v = dec2hex( o.get_frame{ fnd }{ 1 }, 4 );
            o.frame.requested_id = { v( 3:4 ), v( 1:2 ) };
            
            
            % calculate checksum
            cksum = 0;
            fn = fieldnames( o.frame );
            for k = 1:numel( fn )
                if( iscell( o.frame.(fn{k}) ) )
                    cs = size( o.frame.(fn{k}), 2 );
                    for i = 1:cs
                        cksum = cksum + hex2dec( o.frame.(fn{k}){ i } );
                    end
                else
                    cksum = cksum + hex2dec( o.frame.(fn{k}) );
                end
            end
            % split check sum into hex values
            v = dec2hex( cksum, 4 );
            o.frame.checksum = { v( 3:4 ), v( 1:2 ) };
        end
        
        function gen_setframe( o, msg_type, varargin )
            o.frame = o.resetFrame();
            % find msg_type
            % search common frame
            fnd = 0;
            for i = 1:size( o.set_frame, 1 )
                v = o.set_frame{ i };
                if( strcmp( v{ 2 }, msg_type ) )
                    fnd = i;
                    break;
                end
            end
            if( fnd == 0 ), return; end
            
            p = inputParser;
            addRequired( p, 'o', @(x)1 );
            addRequired( p, 'msg_type', @(x)1 );
            for i = 3:length(o.set_frame{ fnd })
                tmp = o.set_frame{ fnd }{ i };
                addParameter( p, string(tmp{ 2 }), [], @(x)1 );
            end
            parse( p, o, msg_type, varargin{:} );
            
            % set message type in frame
            v = dec2hex( o.set_frame{ fnd }{ 1 }, 4 );
            o.frame.message_id = { v( 3:4 ), v( 1:2 ) }; 
            
            % set payload length
            v = dec2hex( o.getFrameLength( msg_type, o.set_frame ), 4 );
%             v = dec2hex( 2, 4 );
            o.frame.payload_length = { v( 3:4 ), v( 1:2 ) };

            payload = {};
            fn = fieldnames( p.Results );
            for i = 1:numel( fn )
                for k = 3:length( o.set_frame{ fnd } )
                    tmp = o.set_frame{ fnd }{ k };
                    if( strcmp( tmp{2}, fn{i} ) )
                        pLength = 0;
                        switch( tmp{1} )
                            case 'u8'
                                pLength = 1;
                            case 'u16'
                                pLength = 2;
                            case 'u32'
                                pLength = 4;
                        end
                        
                        v = dec2hex( p.Results.(fn{i}), 2*pLength );
                        for j = 2*pLength-1:-2:1
                            payload{end+1} = v(j:j+1);
                        end
                    end
                end
            end
            o.frame.requested_id = payload;
            
            
            % calculate checksum
            cksum = 0;
            fn = fieldnames( o.frame );
            for k = 1:numel( fn )
                if( iscell( o.frame.(fn{k}) ) )
                    cs = size( o.frame.(fn{k}), 2 );
                    for i = 1:cs
                        cksum = cksum + hex2dec( o.frame.(fn{k}){ i } );
                    end
                else
                    cksum = cksum + hex2dec( o.frame.(fn{k}) );
                end
            end
            % split check sum into hex values
            v = dec2hex( cksum, 4 );
            o.frame.checksum = { v( 3:4 ), v( 1:2 ) };
        end
        
        function otpt = sendFrame( o )
           fn = fieldnames( o.frame );
           otpt = {};
            for k = 1:numel( fn )
                if( iscell( o.frame.(fn{k}) ) )
                    cs = size( o.frame.(fn{k}), 2 );
                    for i = 1:cs
                        otpt{ end+1 } = o.frame.(fn{k}){ i };
                    end
                else
                    otpt{ end+1 } = o.frame.(fn{k});
                end
            end
            
            for i = 1:numel( otpt )
               write( o.port, hex2dec( otpt{ i } ), "uint8" ); 
            end
        end
        
        function otpt = interpretBuf( o )
            otpt.type = "EMPTY";
            if( isempty( o.buf ) || numel( o.buf ) < 2 )
                return;
            end
            
           % remove everything before the first 'B' and 'R', this is not salvageable 
            while( numel( o.buf ) > 1 )
               if( o.buf( 1 ) == double( 'B' ) )
                   if( o.buf( 2 ) == double( 'R' ) )
                       break;
                   end
               else
                  o.buf = o.buf( 2:end );
               end
               if( numel( o.buf ) <= 0 )
                   return;
               end
            end
            
            if( numel( o.buf ) >= 4 )       % Ensure that there is enough information for length
                MSB = bitshift( o.buf( 4 ), 8 );
                LSB = o.buf( 3 );
                len = MSB + LSB;
                
                if( numel( o.buf ) >= len + 10 )
                    % extract message id
                    msgID = bitshift( o.buf( 6 ), 8 ) + o.buf( 5 );
                    % find matching message id
                    fnd = 0;
                    for i = 1:size( o.get_frame, 1 )
                        v = o.get_frame{ i };
                        if( v{ 1 } == msgID )
                            fnd = i;
                            break;
                        end
                    end
                    if( fnd == 0 ) 
                        % remove the unknown packet from the buffer
                        o = o.clearBuf( 1, 2 );
                        otpt.type = "UNMATCHED";
                        return; 
                    end
                    
                    % ensure that the length of the packet found matches the expected
                    eLen = o.getFrameLength( msgID, o.get_frame );
                    if( eLen ~= len ) 
                        otpt.type = "EMPTY";
%                         o = o.clearBuf( 1, len+10 );
                        return; 
                    end
                    
                    % create structure with the expected output
                    otpt = [];
                    if( isempty( v ) == 0 )
                        otpt.type = v{ 2 };
                    end
                    bufInd = 9;
                    for i = 3:size( v, 2 )
                       if( iscell( v{ i } ) )
                          switch v{ i }{ 1 }
                              case 'u8'
                                  otpt.(v{ i }{ 2 }) = o.buf( bufInd );
                                  bufInd = bufInd + 1;
                              case 'u16'
%                                   otpt.(v{ i }{ 2 }) = bitshift( o.buf( bufInd ), 8 ) + o.buf( bufInd+1 );
                                  t = [ dec2hex( o.buf( bufInd+1 ), 2 ), dec2hex( o.buf( bufInd ), 2 ) ];
                                  otpt.(v{ i }{ 2 }) = hex2dec( t );
                                  bufInd = bufInd + 2;
                              case 'u32'
                                  t = [ dec2hex( o.buf( bufInd+3 ), 2 ), dec2hex( o.buf( bufInd+2 ), 2 ), dec2hex( o.buf( bufInd+1 ), 2 ), dec2hex( o.buf( bufInd ), 2 ) ];
                                  otpt.(v{ i }{ 2 }) = hex2dec( t );
                                  bufInd = bufInd + 4;
                              otherwise
                          end
                       end
                    end
                    
                    
                    send_to_stack = 1;
                    if( ~isempty( o.received_word_func ) )
                        try
                            if( nargout( o.received_word_func )> 0 )
                                send_to_stack = o.received_word_func( otpt );
                            else
                                o.received_word_func( otpt );
                            end
                        catch ME
                            warning( ME );
                        end
                    end
                    
                    if( send_to_stack ), o.received_arr{ end+1 } = otpt; end
                    if( numel( o.received_arr ) > o.max_received_arr_size )
                        o.received_arr = o.received_arr( (end-49):end );
                    end

                    
                    % clear the buffer 
                    o = o.clearBuf( 1, len+10 );
                end
                
                if( len > 700 )
                    % this is a hack to find if an error occured when receiving packets
                    o = o.clearBuf( 1, 2 );
                end
            end
        end
        
       function a = clearBuf( o, sI, eI )
            if( isempty( o.buf ) )              % Ensure that buffer isn't empty
                a = o;
                return;
            end
            
            % Ensure that indices are acceptable and within buffer length
            len = numel( o.buf );
            if( sI >= eI || eI > len || sI >= len )
                a = o;
                return;
            end
            
            % crop and return buffer
            tmp = o.buf( (eI+1):end );
            o.buf = tmp;
            a = o;
       end
       
       function [ ind, frame_name ] = get_key( o, name )
            ind = -1; frame_name = [];
            if( ~isstring( name ) && ~ischar( name ) ), return; end
            if( isstring( name ) ), name = char( name ); end
            
            frames = { 'common_frame', 'get_frame', 'set_frame' };
            
            fnd = 0;
            for i = 1:numel( frames )
                for j = 1:size( o.(frames{i}), 1 )
                    v = o.(frames{i}){j};
                    if( strcmp( v{2}, name ) )
                        fnd = i;
                        frame_name = frames{i};
                        break;
                    end
                end
                if( fnd ), break; end
            end
            if( fnd == 0 ), return; end
            ind = v{1};
       end
       
       function [ name, frame_name ] = get_name_from_key( o, key )
            name = []; frame_name = [];
            if( ~isnumeric( key ) ), return; end
            
            frames = { 'common_frame', 'get_frame', 'set_frame' };
            
            fnd = 0;
            for i = 1:numel( frames )
                for j = 1:size( o.(frames{i}), 1 )
                    v = o.(frames{i}){j};
                    if( isequal( v{1}, key ) )
                        fnd = i;
                        frame_name = frames{i};
                        break;
                    end
                end
                if( fnd ), break; end
            end
            if( fnd == 0 ), return; end
            name = v{2};
       end
    end
    %////////////////////////////////////////////////////////////////////
    % Serial Communication Methods      ->      end
    
    %% Helper Methods
    %////////////////////////////////////////////////////////////////////
    methods
        function get_distance_simple( o )
            o.gen_getframe( 'general_request', 'distance_simple' );
            o.sendFrame();
        end
        
        function get_general_info( o )
            o.gen_getframe( 'general_request', 'general_info' );
            o.sendFrame();
        end
        
        function get_device_id( o )
            o.gen_getframe( 'general_request', 'device_id' );
            o.sendFrame();
        end
        
        function get_speed_of_sound( o )
            o.gen_getframe( 'general_request', 'speed_of_sound' );
            o.sendFrame();
        end
        
        function set_ping_interval( o, interval )
            assert( isnumeric( interval ) );
            o.gen_setframe( 'set_ping_interval', 'ping_interval', interval );
            o.sendFrame();
        end
        
        function set_speed_of_sound( o, ss )
            assert( isnumeric( ss ) );
            o.gen_setframe( 'set_speed_of_sound', 'speed_of_sound', ss );
            o.sendFrame();
        end
        
        function set_device_id( o, did )
            assert( isnumeric( did ) );
            o.gen_setframe( 'set_device_id', 'device_id', did );
            o.sendFrame();
        end
        
        function continuous_start( o, command, interval )
            arguments
                o Ping1D
                command char
                interval double = []
            end
            
            cmd_id = o.get_key( command );
            if( cmd_id == -1 ), return; end
            
            if( ~isempty( interval ) )
                o.set_ping_interval( interval );
            end
            
            o.gen_setframe( 'continuous_start', 'id', cmd_id );
            o.sendFrame();
            
            o.continuous_streams = [ o.continuous_streams, cmd_id ];
        end
        
        function continuous_stop( o, command )
            assert( ischar( command ) );
            
            cmd_id = o.get_key( command );
            if( cmd_id == -1 )
                warning( "Ping1D::continuous_stop: Command not found!" );
                return;
            end
            
            o.gen_setframe( 'continuous_stop', 'id', cmd_id );
            o.sendFrame();
            
            if( ~isempty( o.continuous_streams ) )
                o.continuous_streams = o.continuous_streams( o.continuous_streams ~= cmd_id );
            end
        end
        
        function spectral_stop( o )
            warning( 'Attempting to stop all common continuous streams! This should be used as last resort' );
            
            common_streams = { 'distance_simple', 'distance', 'general_info' };
            for i = 1:numel( common_streams )
                o.continuous_stop( common_streams{i} );
            end
        end
        
        function [ g ] = get_received_by_key( o, key, remove, timeout )
            arguments
                o Ping1D
                key char
                remove logical = true
                timeout double = 10
            end
            
            g = {};
            
            fs = tic; % in case the received array is filling rapidly, this will prevent infinite loop
            i = 1;
            while( i <= numel( o.received_arr ) )
                % get careful subset just in case the received array is being modified is some way
                % asynchronously
                
                v = careful_subset( o.received_arr, 0, 1 );
                if( strcmp( v{1}.type, key ) )
                    g{ end+1 } = v{1};
                    if( remove )
                        o.received_arr(i) = [];
                    else
                        i = i + 1;
                    end
                else
                    i = i + 1;
                end
                
                if( toc( fs ) >= timeout ), break; end
                pause( 0.01 );
            end
        end
        
        function [ otpt ] = get_receipt_by_type_blocking( o, key, remove, timeout )
            arguments
                o Ping1D
                key char
                remove logical = true
                timeout double = 10
            end
            
            otpt = [];
            d = tic;
            while( isempty( otpt ) && toc( d ) <= timeout )
                otpt = o.get_received_by_key( key, remove, timeout ); 
                pause( 0.01 );
            end
        end
        
        function display_stats( o, varargin )
            ind = ismember( varargin( cellfun( @ischar, varargin ) ), '--spectral-stop' );
            if( any( ind ) )
                o.spectral_stop();
                varargin = varargin( ~ind );
            end
            
            p = inputParser;
            addRequired( p, 'o', @(x)(isa(x,'Ping1D')) );
            addOptional( p, 'timeout', 10, @isnumeric );
            addOptional( p, 'clear_buf', false, @islogical );
            addOptional( p, 'info_list', { 'general_info' }, @iscell );
            addOptional( p, 'table', true, @islogical );
            parse( p, o, varargin{:} );
            
            warning( 'Stopping continuous stream! Reactivate afterwards if necessary' );
            for i = 1:numel( o.continuous_streams )
                o.continuous_stop( o.get_name_from_key( o.continuous_streams(i) ) );
            end
            
            if( p.Results.clear_buf )
                o.received_arr = {};
            end
            
            results = {};
            
            % this first iteration will simple print out the results of the info_list
            for i = 1:numel( p.Results.info_list )
                v = {};
                switch( p.Results.info_list{i} )
                    case { 'general_info' }
                        o.get_general_info();
                        v = o.get_receipt_by_type_blocking( 'general_info', true, p.Results.timeout );
                    case { 'device_id' }
                        o.get_device_id();
                        v = o.get_receipt_by_type_blocking( 'device_id', true, p.Results.timeout );
                    case { 'speed_of_sound' }
                        o.get_speed_of_sound();
                        v = o.get_receipt_by_type_blocking( 'speed_of_sound', true, p.Results.timeout );
                    otherwise
                        v = { "Not defined" };
                end
                
                if( ~isempty( v ) )
                    % get the last packet and place into results cell
                    if( p.Results.table )
                        fn = fieldnames( v{ end } );
                        fn = fn( find( fn ~= "type" ) );
                        for j = 1:numel( fn )
                            results{end+1} = { fn{j}, v{ end }.(fn{j}) };
                        end
                    else
                        disp( v{ end } );
                    end
                end
            end
            
            if( ~isempty( results ) && p.Results.table )
                tt = TextTable();
                tt.table_title = 'Ping Sonar Info.';
                tt.addColumns( { 'Characteristic', 'Value' }, [ -1 -1 ],  [ 2 2 ] );
                for j = 1:numel( results )
                    tt.addRow( { string( results{j}{1} ) string( results{j}{2} ) } );
                end
                
                tt.print( true, false );
            end
        end
    end
    %////////////////////////////////////////////////////////////////////
    % Helper Methods      ->      end
    
    methods ( Static )
        function cb( src, ~ )
            data = read( src, 1, "uint8" );
            src.UserData( 'UpdateRec', ToggleEventData( data ) );
        end
        
        %%  Information Struct Constructor
        %////////////////////////////////////////////////////////////////////
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Usage: resetFrame( {class} )
        %
        % Function to simulate typedef struct in c++. Call this function
        % to return a zero/null/empty initialized structure with the information
        % which can be stored about the frame data for the XBee. This
        % function is used to create repeatable structure definitions which
        % seems to be functionality which MATLAB is lacking.
        %
        % Input     None
        %
        % Output    o :         Structure used to house XBee
        %                           frame data
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [ fr ] = resetFrame()
            %https://docs.bluerobotics.com/ping-protocol/#request-protocol-version
            fr = struct( 'start1', dec2hex( 'B' ), 'start2', dec2hex( 'R' ), 'payload_length', [], ...
                'message_id', [], 'src_device_id', '00', 'dst_device_id', '00', 'requested_id', [], ...
                    'checksum', '00' );
        end
        %////////////////////////////////////////////////////////////////////
    
        function na( varargin )
            
        end
    end
    
end