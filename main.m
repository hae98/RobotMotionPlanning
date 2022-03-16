% ROBOT MOTION PLANNING

clc;clf
clear all
close all

% load workspace and obstacle data
load workspace_data

% create polygon containing holes representing the workspace and obstacles
x = [];
y = [];
for i = 1:length(workspace)
    x = [x, cell2mat(workspace{i}(:,1))', nan];  % NaN separates obstacles
    y = [y, cell2mat(workspace{i}(:,2))', nan];
end
wksp_pgon = polyshape(x,y); 

% plot workspace
f = figure;
plot(wksp_pgon, 'FaceColor', 'w', 'FaceAlpha', 1)
set(gca, 'Color', 'k')
xlim([0 200])
ylim([0 200])
hold on

% start and goal points user input
in_wksp = 0;
obstacle_check = 0;
while any([in_wksp,all(obstacle_check == 0)] == 0)
    start = input('Enter a start point with values between 0 and 200 (e.g., [50 10]): ');
    if any([start(1,1),start(1,2)] > 200) || any([start(1,1),start(1,2)] < 0)
        disp('Error: Start point is outside of workspace. Please enter a different point.')
        fprintf('\n')
        in_wksp = 0;
    else
        in_wksp = 1;
    end
    obstacle_check = [];
    for i = 1:wksp_pgon.NumHoles
        [x,y] = boundary(wksp_pgon,i+1);
        [in_s,on_s] = inpolygon(start(1),start(2),x,y);
        if any([in_s; on_s] == 1)
            disp('Error: Start point is in or on an obstacle. Please choose a different point.')
            fprintf('\n')
            obstacle_check = [obstacle_check; 1];
        else
            obstacle_check = [obstacle_check; 0];
        end
    end
end

in_wksp = 0;
obstacle_check = 0;
while any([in_wksp,all(obstacle_check == 0)] == 0)
    goal = input('Enter a goal point with values between 0 and 200 (e.g., [50 10]): ');
    if any([goal(1,1),goal(1,2)] > 200) || any([goal(1,1),goal(1,2)] < 0)
        disp('Error: Goal point is outside of workspace. Please enter a different point.')
        fprintf('\n')
        in_wksp = 0;
    else
        in_wksp = 1;
    end
    obstacle_check = [];
    for i = 1:wksp_pgon.NumHoles
        [x,y] = boundary(wksp_pgon,i+1);
        [in_s,on_s] = inpolygon(goal(1),goal(2),x,y);
        if any([in_s; on_s] == 1)
            disp('Error: Goal point is in or on an obstacle. Please choose a different point.')
            fprintf('\n')
            obstacle_check = [obstacle_check; 1];
        else
            obstacle_check = [obstacle_check; 0];
        end
    end
end

% plot start and goal points
plot([start(1), goal(1)], [start(2), goal(2)], "b.", 'MarkerSize', 18)
text(start(1)+2,start(2),'Start','Color','b','FontSize', 13)  % label start point
text(goal(1)+2,goal(2),'Goal','Color','b','FontSize', 13)  % label goal point

 % decompose workspace into collection of trapezoids
[T] = sweeping_trapezoidation_algorithm(wksp_pgon);

% obtain roadmap in form of adjacency table
[AdjTable,nodes,start_node,goal_node] = roadmap_from_decomposition_algorithm(T,start,goal);

% generate path using breadth-first search algorithm
[path] = BFS_algorithm(AdjTable, start_node, goal_node);

% path as points
P = [start; nodes(path,:); goal];

% figure size and position
set(gcf, 'units', 'points', 'position', [10 10 650 500])
title ('Robot Motion Planning')

% animate path
h = animatedline('Color','red','Linewidth',2);
filename = 'robot_motion_planning_plot.gif';   % filename of gif
check_flag = 0;
for k = 1:length(P)
    addpoints(h,double(P(k,1)), double(P(k,2)));
    drawnow 
    pause(0.12)
    
    % capture the plot as an image
    frame = getframe(f);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    
    % write to GIF file
    if check_flag == 0
        imwrite(imind,cm,filename,'gif','LoopCount',inf);
        check_flag = 1;
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append');
    end
end






