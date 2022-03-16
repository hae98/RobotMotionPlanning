function [T] = sweeping_trapezoidation_algorithm(wksp_pgon)
% Input: a polygon possibly with polygonal holes 
% Output: a set of trapezoids, whose union equals the polygon

obstacles = {};
numObstacles = wksp_pgon.NumHoles;
[wksp_x,wksp_y] = boundary(wksp_pgon,1);
workspace = [wksp_x(1:end-1,:) wksp_y(1:end-1,:)];

first_seg = workspace((workspace(:,2) == min(unique(workspace(:,2)))),:);
last_seg = workspace((workspace(:,2) == max(unique(workspace(:,2)))),:);

% get obstacle vertices from polyshape
seg{1} = [first_seg];
for i = 1:numObstacles
    [x,y] = boundary(wksp_pgon,i+1);
    obstacles{i} = [x(1:end-1,:) y(1:end-1)];
    
    % obstacles segments
    for j = 1:length(obstacles{i})  % for each obstacles vertices
        if j == length(obstacles{i})
            seg{end+1} = [obstacles{i}(j,1), obstacles{i}(j,2);obstacles{i}(1,1),obstacles{i}(1,2)];
        else
            seg{end+1} = [obstacles{i}(j,1),obstacles{i}(j,2);obstacles{i}(j+1,1),obstacles{i}(j+1,2)];
        end
    end
end
seg{end+1} = [last_seg];

% sort segments by endpoints
for i = 1:length(seg)
     if seg{i}(1,1) > seg{i}(2,1)
        temp1 = seg{i}(1,:);
        temp2 = seg{i}(2,:);
        seg{i}(1,:) = temp2;
        seg{i}(2,:) = temp1;
    end
end

T = {};  % initialize empty list T of trapezoids
midpoints = [];

% order all vertices by increasing x coordinate
x_obs = [];
y_obs = [];

for i = 1:numObstacles
    x_obs = [x_obs; obstacles{i}(:,1)];
    y_obs = [y_obs; obstacles{i}(:,2)];
end
[x_obs_sort, x_obs_order] = sort(x_obs);
y_obs_sort = y_obs(x_obs_order);
V = [x_obs_sort y_obs_sort];

for i = 1:length(V)  % for each vertex
    v = V(i,:);  % current vertex
    S = [];  % maintain list s of obstacle segments intersected by sweeping line L
    int_pts = [];  % stores intersection points 
    seg_int_id = [];
    trap = [];
    S_n = [];
    
    % sweeping line L
    L_t = [v(1,1), 0];
    L_b = [v(1,1), 200];
    
    % determine which obstacle segments are intersected by L
    for n = 1:length(seg)
        [int, p_int] = doTwoSegmentsIntersect(L_t,L_b,seg{n}(1,:),seg{n}(2,:));
        if int
            S_n = [n; S_n];
            S = [seg{n}(1,:);seg{n}(2,:); S];
            int_pts = [p_int; int_pts];
        end
    end
    
    % number of obstacle segments in S that have v as an endpoint
    P = []; 
    v_l = [];
    v_r = [];
    
    for j = 1:length(S_n)
        isPresent = ismember(seg{S_n(j)}, v, 'rows');
        if isPresent(2)  % if v is a right endpoint remove from S
            v_l = [v_l; seg{S_n(j)}(1,:)];  % corresponding left endpoint
            S_n(j) = nan;
        elseif isPresent(1)  % if v is a left endpoint
            v_r = [v_r; seg{S_n(j)}(2,:)];  % cooresponding right endpoint
        end
    end
    P = [v_l; v_r];
      
    % convextivity test
    vec_1 = [P(1,1),P(1,2),0] - [v(1,1), v(1,2), 0];
    vec_2 = [P(2,1),P(2,2),0] - [v(1,1), v(1,2), 0];
    CosTheta = max(min(dot(vec_1,vec_2)/(norm(vec_1)*norm(vec_2)),1),-1);
    theta = real(acosd(CosTheta));
    
    % determine type of vertex
    if theta < 180 && size(v_r,1) == 2
        type = 1;
        if isinterior(wksp_pgon, v(1,1)+1, v(1,2))
            temp = theta;
            theta = 360 - temp;
            type = 2;
        end    
    elseif theta > 180 && size(v_r,1) == 2
        type = 2;
    elseif theta < 180 && size(v_l,1) == 2
        type = 3;
        if ~isinterior(wksp_pgon, v(1,1)+1, v(1,2)) 
            temp = theta;
            theta = 360 - temp;
            type = 4;
        end    
    elseif theta > 180 && size(v_l,1) == 2
        type = 4;
    elseif theta < 180 && size(v_r,1) == 1
        type = 5;
    elseif theta > 180 && size(v_r,1) == 1
        type = 6;
    end
   
    % pt and pb
    greater_rows = find(round(int_pts(:,2)) > v(1,2)); 
    min_greater_row = min(int_pts(greater_rows, 2));
    pt_row = find(int_pts(:,2) == min_greater_row);
    pt = int_pts(pt_row,:);
    pt_id = S_n(pt_row);
    pt_seg = seg{pt_id};
    pt_l = pt_seg(1,:);
   
    lesser_rows = find(round(int_pts(:,2)) < v(1,2));
    max_lesser_row = max(int_pts(lesser_rows, 2));
    pb_row = find(int_pts(:,2) == max_lesser_row);
    pb = int_pts(pb_row,:);
    pb_id = S_n(pb_row);
    pb_seg = seg{pb_id};
    pb_l = pb_seg(1,:);

    % add to T zero, one, or two new trapezoids depending on vertex type
    if type == 1
        T{end+1} = [v; pt; pt_l; pb_l; pb];
    elseif type == 2
        pt = nan;
        pb = nan;
    elseif type == 3
        [~,max_row] = max(v_l(:,2));
        [~,min_row] = min(v_l(:,2));   
        T{end+1} = [pt; pt_l; v_l(max_row,:); v];
        T{end+1} = [pb; pb_l; v_l(min_row,:); v];
    elseif type == 4
        T{end+1} = [pt; pt_l; pb_l; pb];
        pt = nan;
        pb = nan;
    elseif type == 5
        if isinterior(wksp_pgon, v(1,1), v(1,2)+1) % if point above v is in workspace polygon
            T{end+1} = [v; v_l; pt_l; pt];
            pb = nan;
        else
            T{end+1} = [v; v_l; pb_l; pb];
            pt = nan;
        end
    elseif type == 6
        T{end+1} = [v; v_l; pb_l; pb];
        pb = nan;
    end
    
    % update left endpoints of the obstacle segments
    if ~isnan(pt)
        seg{pt_id}(1,:) = pt;  % update pt segment
        midpoints = [midpoints; [pt(1,1), (pt(1,2)+v(1,2))/2]];
    end
    if ~isnan(pb)
        seg{pb_id}(1,:) = pb;  % update pb segment
        midpoints = [midpoints; [pb(1,1), (v(1,2)+pb(1,2))/2]];
    end
    
    % last trapzoid
    if i == length(V)
        T{end+1} = [pt; pb; first_seg(2,:); last_seg(2,:)];
    end
end