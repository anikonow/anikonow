function [path,costMatrix] = Gtsp(nodes, polygon)

%
% N x 2 matrix with the coordinates of the cities
%

%% Setup
%Number or nodes
N = size(nodes, 1);

%cost matrix
costMatrix = zeros(N, N);
for i = 1:N
    for j = 1:N
        costMatrix(i, j) = norm(nodes(i, :) - nodes(j, :));
    end
end

%nodes with 0 cost (within cell)
p = 1;
for k = 1:2:N
    zeroCostPairs(p,:) = [k, k+1];
    p = p + 1;
end

for k = 1:size(zeroCostPairs, 1)
    i = zeroCostPairs(k, 1);
    j = zeroCostPairs(k, 2);
    costMatrix(i, j) = 0;
    costMatrix(j, i) = 0; 
end

% Check if any segment between nodes crosses outside the polygon
for i = 1:N
    for j = i+1:N
        if ~isSegmentInsidePolygon(nodes(i, :), nodes(j, :), polygon)
            % If the segment crosses outside the polygon
            costMatrix(i, j) = costMatrix(i, j) * 1000;
            costMatrix(j, i) = costMatrix(j, i) * 1000;
        end
    end
end

%heuristic
currentNode = 1;  % Starting node
visitedNodes = false(N, 1);
visitedNodes(currentNode) = true;
path = currentNode;

while sum(visitedNodes) < N
    % Find the nearest unvisited node
    distances = costMatrix(currentNode, :);
    distances(visitedNodes) = inf;  % Ignore already visited nodes
    [~, nearestNode] = min(distances);
    path = [path, nearestNode];
    visitedNodes(nearestNode) = true;
    currentNode = nearestNode;
end

% % Return to start
% %path = [path, path(1)];
% %minCost = calculateCost(path); % calculate cost

% Display the results
% fprintf('Path: ');
% disp(path);


% Plot the approximate path
% figure;
% plot(nodes(:,1), nodes(:,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
% hold on;
% plot(nodes(path, 1), nodes(path, 2), 'r--o');
% set(gca,'YDir','reverse')
% xlabel('X');
% ylabel('Y');
% title('GTSP Path');

end

function isInside = isSegmentInsidePolygon(p1, p2, polygon)
    % Check if the line segment between p1 and p2 stays inside the polygon
    numPoints = 100; % Number of points to check along the segment
    xVals = linspace(p1(1), p2(1), numPoints);
    yVals = linspace(p1(2), p2(2), numPoints);
    isInside = all(inpolygon(xVals, yVals, polygon(:, 1), polygon(:, 2)));
end