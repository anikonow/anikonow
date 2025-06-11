clc
clear

%% Change file location and
%% Change file name


outputFilename = 'Data\test25\surveyA1_low_raw.txt';
filename = 'Test_25\20250328_120521_Dual_30kHz_id_445_nmea.log'; % 
fid = fopen(filename, 'r');

frequency = [];
timestamps = {};
depths_m = [];
temperatures_C = [];
pitch_roll = [];
gnss_data = struct('time', {}, 'latitude', {}, 'longitude', {}, 'altitude', {});

while ~feof(fid)
    line = fgetl(fid);
    
    % Parse frequency (#F lines)
    if startsWith(line, '#F')
        freq = sscanf(line, '#F %f Hz');
        frequency(end+1) = freq; %#ok<SAGROW>
    
    % Parse time ($SDZDA lines)
    elseif startsWith(line, '$SDZDA')
        parts = split(line, ',');
        timestamps{end+1} = parts{2}; %#ok<SAGROW>
    
    % Parse depth ($SDDBT lines)
    elseif startsWith(line, '$SDDBT')
        parts = split(line, ',');
        depth = str2double(parts{4}); % Depth in meters
        depths_m(end+1) = depth; %#ok<SAGROW>
    
    % Parse temperature ($SDMTW lines)
    elseif startsWith(line, '$SDMTW')
        parts = split(line, ',');
        temp = str2double(parts{2}); % Temperature in Celsius
        temperatures_C(end+1) = temp; %#ok<SAGROW>
    
    % % Parse pitch and roll ($SDXDR lines)
    % elseif startsWith(line, '$SDXDR')
    %     parts = split(line, ',');
    %     pitch = str2double(parts{2}); % Pitch
    %     roll = str2double(parts{6}); % Roll
    %     pitch_roll(end+1, :) = [pitch, roll]; %#ok<SAGROW>
    
    % Parse GNSS data ($GPGGA lines)
    elseif startsWith(line, '$GPGGA')
        parts = split(line, ',');
        time = parts{2};
        lat = convert_to_decimal(parts{3}, parts{4});
        lon = convert_to_decimal(parts{5}, parts{6});
        altitude = str2double(parts{10});
        gnss_data(end+1) = struct('time', time, 'latitude', lat, 'longitude', lon, 'altitude', altitude); %#ok<SAGROW>
    end
end
fclose(fid);

maxLength = max([length(depths_m), length(temperatures_C), length(gnss_data)]);
depths_m = padarray(depths_m(:), maxLength - length(depths_m), NaN, 'post');
temperatures_C = padarray(temperatures_C(:), maxLength - length(temperatures_C), NaN, 'post');

gnss_time = strings(maxLength, 1);
gnss_lat = nan(maxLength, 1);
gnss_lon = nan(maxLength, 1);
gnss_alt = nan(maxLength, 1);

%% Corrections;
gnss_alt=gnss_alt-.66;
depths_m=depths_m+.2349;


for i = 1:length(gnss_data)
    gnss_time(i) = gnss_data(i).time;
    gnss_lat(i) = gnss_data(i).latitude;
    gnss_lon(i) = gnss_data(i).longitude;
    gnss_alt(i) = gnss_data(i).altitude;
end

T = table(gnss_time, gnss_lat, gnss_lon, gnss_alt, depths_m, temperatures_C, ...
    'VariableNames', {'GNSS_Time', 'GPSLatitude', 'GPSLongitude', 'Altitude', 'Depth_m', 'Temperature_C'});

%output file
writetable(T, outputFilename);

%Confirm the file has been written
fprintf('Data successfully written to %s\n', outputFilename);

function decimal = convert_to_decimal(coord, direction)
    degrees = floor(str2double(coord) / 100);
    minutes = mod(str2double(coord), 100);
    decimal = degrees + minutes / 60;
    if direction == 'S' || direction == 'W'
        decimal = -decimal;
    end
end