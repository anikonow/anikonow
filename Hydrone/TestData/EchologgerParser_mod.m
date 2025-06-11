clc
clear

%% Change file location and
%% Change file name


outputFilename = 'Data\test26\surveyB1_high.txt';
filename = 'Test_26\20250408_133711_Dual_200kHz_id_445_nmea.log';%'hydrone/TestData/Test_22/Test22_low.csv'; % 
%filename = '20250124_125109_Dual_30kHz_id_445_nmea.log';%'hydrone/TestData/Test_22/Test22_low.csv'; % 

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
gnss_alt=gnss_alt-.7;
depths_m=depths_m+.2349;

depth_threshold_max = 10;
depth_threshold_min = 0.1;
invalid_bool1 = (depths_m > depth_threshold_max);
invalid_bool2 = (depths_m < depth_threshold_min) ;

invalid_ind = find(invalid_bool1+invalid_bool2>=1);

for i = 1:length(gnss_data)
    gnss_time(i) = gnss_data(i).time;
    gnss_lat(i) = gnss_data(i).latitude;
    gnss_lon(i) = gnss_data(i).longitude;
    gnss_alt(i) = gnss_data(i).altitude;
end

figure;
subplot(2,1,1)
plot(depths_m,'r')
hold on;

% remove invalid 
gnss_time(invalid_ind) = [];
gnss_lat(invalid_ind) = [];
gnss_lon(invalid_ind) = [];
gnss_alt(invalid_ind) = [];
temperatures_C(invalid_ind) = [];
depths_m(invalid_ind) = [];

subplot(2,1,2)
plot(depths_m,'b')
hold on;


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