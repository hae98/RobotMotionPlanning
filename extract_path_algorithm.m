function [path, path_exist] = extract_path_algorithm(goal_node, parent)
% input: a goal node and parent values
% output: a path from start node to goal node

path = [goal_node];
u = goal_node;
while parent{u} ~= u
    u = parent{u};
    path = [u; path];
    if isnan(parent{goal_node})
        error('Path between start and goal node does not exist.')
    end
end