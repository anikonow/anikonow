%% intelrobotics HW4 Astar %%

clc
clear

% Config Space
grid_size = 200;
num_obstacles = 2000;
start_pos = [50, 50];
goal_pos = [150, 150];

% Create Obstacles
grid = zeros(grid_size, grid_size); 
for i = 1:num_obstacles
    x = randi(grid_size);
    y = randi(grid_size);
    grid(x, y) = 1; 
end
close all


path = astar(grid_size, start_pos, goal_pos, grid);

if ~isempty(path)
    disp('Path found :)');
    close all;
    plot_path(start_pos, goal_pos, path, grid);
else
disp('No path found :(');
end


%A*
function path = astar(grid_size, start_pos, goal_pos, grid)

    snode = {start_pos};
    node = containers.Map();

    cost = containers.Map();
    cost(num2str(start_pos)) = 0;
    
    % A* loop
    while ~isempty(snode)
     %Current pos
     c_pos = snode{1};
     snode(1) = [];
        
     % Check if reached the goal
     if isequal(c_pos, goal_pos)
     path = reconpath(node, goal_pos);
     return;
     end


     %Check next node
     nnode = find(c_pos, grid_size);
     for i = 1:size(nnode, 1)
            neighbor_pos = nnode(i, :);

            if grid(neighbor_pos(1), neighbor_pos(2)) == 1
            continue;
            end
            
            %Cost 
            min = cost(num2str(c_pos)) + 1;
            
            % Update if path cost is lower
        if ~cost.isKey(num2str(neighbor_pos)) || min < cost(num2str(neighbor_pos))
                
        node(num2str(neighbor_pos)) = c_pos;
        cost(num2str(neighbor_pos)) = min;
               
        if ~any(cellfun(@(pos) isequal(pos, neighbor_pos), snode))
        snode{end+1} = neighbor_pos;
        end
      end
    end
   end
    
    % No path found
    path = [];
end
% 
% %Cost
% function h = heuristic_cost(pos, goal_pos)
%     h = abs(pos(1) - goal_pos(1)) + abs(pos(2) - goal_pos(2));
%    % h=norm2(pos(1) - goal_pos(1)) + abs(pos(2) - goal_pos(2));
% end

%Create and check next node +/- 1
function neighbors = find(pos, grid_size)
    d = [0,1;0,-1; 1,0;-1,0];
    neighbors = pos + d;

    
    valid = all(neighbors >= 1, 2) & all(neighbors <= grid_size, 2);
    neighbors = neighbors(valid, :);
end

% Reconstuct Path Post Algo
function path = reconpath(lastnode, goal_pos);
    path = [];
    currentpos = goal_pos;
    
    while lastnode.isKey(num2str(currentpos));
    path = [currentpos; path];

    currentpos = lastnode(num2str(currentpos));
    end
    path = [currentpos; path];
end


% Plot A*
function plot_path( startpos, goalpos, path, grid)
    
    figure;
    imagesc(grid);
    colormap([1 1 1; 0 0 0]); 
    hold on;
    
    %Plot start and goal
    plot(startpos(2), startpos(1), 'go', 'MarkerSize',10,'MarkerFaceColor', 'g');
    text(startpos(2), startpos(1), 'Start');
    plot(goalpos(2), goalpos(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(goalpos(2), goalpos(1), 'Goal');
  
    if ~isempty(path)
    x_path = path(:, 2);
    y_path = path(:, 1);
    plot(x_path, y_path, 'b-', 'LineWidth', 2);
    end
    
    axis equal;
    xlabel('Columns');
    ylabel('Rows');
    title('A*');
end






