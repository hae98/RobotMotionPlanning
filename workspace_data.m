% workspace and obstacle data

% workspace boundary
area(:,1) = [0, 0, 200, 200];  
area(:,2) = [0, 200, 200, 0];  

% obstacles
obstacle_1(:,1) = [85; 100; 80; 65];
obstacle_1(:,2) = [15; 40; 60; 35];

obstacle_2(:,1) = [35, 60, 55, 30];
obstacle_2(:,2) = [70, 75, 105, 100];

obstacle_3(:,1) = [125, 150, 130, 105];
obstacle_3(:,2) = [95, 120, 145, 120];

obstacle_4(:,1) = [122, 110, 50, 120, 180, 165, 129, 90];
obstacle_4(:,2) = [160, 180, 130, 40, 90, 110, 80, 130];

obstacles{:,:,1} = num2cell(obstacle_1);
obstacles{:,:,2} = num2cell(obstacle_2);
obstacles{:,:,3} = num2cell(obstacle_3);
obstacles{:,:,4} = num2cell(obstacle_4);

% complete workspace
workspace{:,:,1} = num2cell(area);
workspace{:,:,2} = num2cell(obstacle_1);
workspace{:,:,3} = num2cell(obstacle_2);
workspace{:,:,4} = num2cell(obstacle_3);
workspace{:,:,5} = num2cell(obstacle_4);

save('workspace_data.mat')








 
