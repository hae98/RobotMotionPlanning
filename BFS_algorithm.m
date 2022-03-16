function [path] = BFS_algorithm(AdjTable, start_node, goal_node)
% input: a graph in the form of an adjacency table, a start node, and a
% goal node
% output: a path from start node to goal node if it exists, otherwise a
% failure notice
 
parent = {};
for i = 1:length(AdjTable)
    parent{i} = nan;
end
parent{start_node} = start_node;
% create empty queue and insert start node
Q = [];
Q = [Q; start_node];
while ~isempty(Q)
    % retrieve item that sits at front of queue
    v = Q(1);
    Q(1) = [];
    % for each node u connected to v by an edge
    for u = AdjTable{v}
        if isnan(parent{u})
            parent{u} = v;
            Q = [Q; u];
        end
        if u == goal_node
            % run extract-path alogorithm to extract path
            [path] = extract_path_algorithm(goal_node, parent);
        end
    end
end
