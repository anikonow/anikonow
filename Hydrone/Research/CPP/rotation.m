function [x, y] = rotation(easting, northing)
    % Create figure
    fig = figure('Name', 'Rotation Demo', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 600, 500]);

    x = easting;
    y = northing;
    
    % Create axes for plotting
    ax = axes('Parent', fig);
    hold(ax, 'on');
    axis equal;
    grid on;
    
    % Plot original shape (Reference)
    originalPlot = plot(ax, x, y, 'k--', 'LineWidth', 1.5); 
    rotatedPlot = plot(ax, x, y, 'b', 'LineWidth', 2); % This will update

    % Create text label to display angle **BEFORE** the slider callback is defined
    angleText = uicontrol('Style', 'text', 'String', '0°', ...
        'Units', 'normalized', 'Position', [0.85, 0.05, 0.1, 0.03], ...
        'FontSize', 12, 'FontWeight', 'bold');

    % Create slider
    slider = uicontrol('Style', 'slider', 'Min', -180, 'Max', 180, ...
        'Value', 0, 'Units', 'normalized', ...
        'Position', [0.2, 0.05, 0.6, 0.05], ...
        'Callback', @(src, ~) updatePlot(src.Value, x, y, rotatedPlot, angleText));

    % Add label for slider
    uicontrol('Style', 'text', 'String', 'Rotation Angle (degrees)', ...
        'Units', 'normalized', 'Position', [0.4, 0.01, 0.2, 0.03]);

    % Initial update
    updatePlot(0, x, y, rotatedPlot, angleText);
end

function updatePlot(angle, x, y, plotHandle, angleText)
    % Convert angle to radians
    theta = deg2rad(angle);

    % Rotation matrix
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % Apply transformation
    rotatedPoints = R * [x'; y'];

    % Update angle text
    set(angleText, 'String', sprintf('%.1f°', angle));

    % Update plot
    set(plotHandle, 'XData', rotatedPoints(1, :), 'YData', rotatedPoints(2, :));
end