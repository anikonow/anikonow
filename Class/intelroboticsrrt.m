%% intelrobotics HW4 RRT %%

clc
clear

% Config Space
grid_size = 200;
num_obstacles = 6000; 
start_pos = [50, 50]; 
goal_pos = [150, 150]; 

% Create Obstacles
grid = zeros(grid_size, grid_size); 
for i = 1:num_obstacles
    x = randi(grid_size);
    y = randi(grid_size);
    grid(x, y) = 1; 
end

[tree, path] = rrt(grid_size, num_obstacles, start_pos, goal_pos, grid);

if ~isempty(path)
    disp('Path found :) :');
    disp(path);
    close all


    plot_rrt( start_pos, goal_pos, tree, path, grid);
else
    disp('No path found :(');
end



% RRT
function [tree, path] = rrt(grid_size, num_obstacles, start_pos, goal_pos, grid)
    tree = struct('par', [], 'pos', start_pos);
    
    % Iteration Parameters
    i = 20000;
    ss = 1; % step size
    gr = 10; %radius of goal
    
  
    for iter = 1:i
        
    %Creat Branches
    rand_point = [randi(grid_size), randi(grid_size)];

   node = findnode(tree, rand_point);
   new_point = tdist(node.pos, rand_point, ss);
        
      % Check if next point is valid
      if find(grid, new_point)
            neztnode.par = node;
            neztnode.pos = new_point;
            tree(end+1) = neztnode;
            
            % Check if the goal is reached
          if norm(new_point - goal_pos) <= gr
          path = reconpath(tree, numel(tree));
          return;
          end
      end
    end
% No path found
path = []; 
end

% Find Nearest Node
function node = findnode(tree, point)
    distances = arrayfun(@(node) norm(node.pos - point), tree);
    [~, j] = min(distances);

    node = tree(j);
end

%New point
function point =tdist(current_point, target_point,step_size)
    dir =target_point -current_point;
    d = norm(dir);
    %travel toward point
    if d <= step_size
    point = target_point;
    else
    point =current_point+(step_size/d)*dir;
    end
end

% Reconstruct Path
function path = reconpath(tree, goal_idx)
    path = [];
    currentnode = tree(goal_idx);

    
    while ~isempty(currentnode)

        path = [path; currentnode.pos];
        currentnode = currentnode.par;
    end
    path = flipud(path); 
end

function neighbors = find(grid, point)
    x = round(point(1));
    y = round(point(2));
    neighbors = x >= 1 && x <= size(grid, 1) && y >= 1 && y <= size(grid, 2) && grid(x, y) == 0; 
end

% Plot RRT
function plot_rrt(start_pos,goal_pos,tree, path, grid)
    figure;
    imagesc(grid);
    colormap([1 1 1; 0 0 0]);
    hold on;
    
    % Plot start and goal
    plot(start_pos(2), start_pos(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(start_pos(2), start_pos(1), 'Start');
    plot(goal_pos(2), goal_pos(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(goal_pos(2), goal_pos(1), 'Goal');
    
    % Plot RRT branches
    for i = 2:numel(tree)
        tnodes = tree(i);
        p = tnodes.par;
        if ~isempty(p)
        plot([p.pos(2), tnodes.pos(2)], [p.pos(1), tnodes.pos(1)], 'k-', 'LineWidth', 1);
        end
    end
    
    % Plot path
    if ~isempty(path)
        xpath = path(:, 1);
        ypath = path(:, 2);
        plot(ypath, xpath, 'b-', 'LineWidth', 2);
    end
    
    axis equal;
    xlabel('Columns');
    ylabel('Rows');
    title('RRT');
end







