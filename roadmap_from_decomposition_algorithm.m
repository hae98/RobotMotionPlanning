function [AdjTable,nodes,start_node,goal_node] = roadmap_from_decomposition_algorithm(T,start,goal)
% input: free workspace, start point, goal point
% output: a roadmap represented as an adjacency table, list of nodes, the
% start node, and goal node

% find and plot midpoints 
midpoints = [];
for i = 1:length(T)
    rows = find(T{i}(:,1) == (max(T{i}(:,1))));
    s = T{i}(rows,:);
    for j = 1:length(s)-1
        p1 = s(1,:);
        p2 = s(j+1,:);
        if p1(1,1) == p2(1,1) && p1(1,1) ~= 0 && p1(1,1) ~= 200
            midpoints = [midpoints; [p1(1,1), (p1(1,2)+p2(1,2))/2]];
        end
    end
end
plot(midpoints(:,1), midpoints(:,2), '.', 'Color', 'k', 'MarkerSize', 12)


% find and plot centroids
centroids = [];  
for i = 1:length(T)
    [c_x, c_y] = centroid(polyshape(T{i}(:,1), T{i}(:,2)));
    centroids = [centroids; c_x c_y];
end
plot(centroids(:,1), centroids(:,2), 'd', 'Color', 'k')

% for each trapezoid connect the center to all the midpoints
start_T = [];
goal_T = [];
nodes = [];

for i = 1:length(T)
    
    % plot trapezoids
    plot(polyshape(T{i}(:,1),T{i}(:,2)),'FaceColor','w')
    
    edge = [];
    nodes = [nodes; centroids(i,:)];
    AdjTable{length(nodes)} = [];
    
    % find all midpoints located on current trapezoid
    [mid_in, mid_on] = inpolygon(midpoints(:,1), midpoints(:,2), T{i}(:,1), T{i}(:,2));
    in_on = [mid_in, mid_on];
    T_midpoints = midpoints(find(ismember(in_on, [1, 1], 'rows')),:);
    
    % connections between centers and midpoints
    for j = 1:size(T_midpoints,1)
        if T_midpoints(j,1) > centroids(i,1)
            nodes = [nodes; T_midpoints(j,:)];
            AdjTable{length(nodes)} = [];
        end
        p1 = T_midpoints(j,:);
        p2 = centroids(i,:);
        n1 = find(ismember(nodes, p1,'rows'));
        n2 = find(ismember(nodes, p2,'rows'));
        AdjTable{n1}(end+1) = n2;
        AdjTable{n2}(end+1) = n1;
        plot([p1(1,1), p2(1,1)],[p1(1,2), p2(1,2)], '--', 'Color','k')
    end
    
    % connect start and goal points to roadmap 
    [start_in, start_on] = inpolygon(start(1), start(2), T{i}(:,1), T{i}(:,2));
    [goal_in, goal_on] = inpolygon(goal(1), goal(2), T{i}(:,1), T{i}(:,2));  
    if start_in == 1 && start_on == 0    
        plot([centroids(i,1) start(1)], [centroids(i,2) start(2)], '--', 'Color', 'r')
        start_centroid = centroids(i,:);
        start_trap = i;
    end
    if goal_in == 1 && goal_on == 0    
        plot([centroids(i,1) goal(1)], [centroids(i,2) goal(2)], '--', 'Color', 'r')
        goal_centroid = centroids(i,:);
        goal_trap = 1;
    end
    
    % if start or goal on vertical segment store corresponding trapezoids
    if start_in == 1 && start_on == 1
        start_T = [start_T; i];
    end
    if goal_in == 1 && goal_on == 1
        goal_T = [goal_T; i];
    end
end

% if start on vertical segment connect to centroid of minimum distance
if size(start_T) > 0
    d_start = [pdist([centroids(start_T(1),:); start], 'euclidean');pdist([centroids(start_T(2),:); start], 'euclidean')];
    start_trap = start_T(find(d_start == min(d_start)));
    start_centroid = centroids(start_trap,:);
    plot([centroids(start_trap,1) start(1)], [centroids(start_trap,2) start(2)], '--', 'Color', 'r');
end
% if goal on vertical segment connect to centroid of minimum distance
if size(goal_T) > 0
    d_goal = [pdist([centroids(goal_T(1),:); goal], 'euclidean');pdist([centroids(goal_T(2),:); goal], 'euclidean')];
    goal_trap = goal_T(find(d_goal == min(d_goal)));
    goal_centroid = centroids(goal_trap,:);
    plot([centroids(goal_trap,1) goal(1)], [centroids(goal_trap,2) goal(2)], '--', 'Color', 'r');
end

% identify start and goal node
start_node = find(ismember(nodes, start_centroid,'rows'));
goal_node = find(ismember(nodes, goal_centroid,'rows'));

    