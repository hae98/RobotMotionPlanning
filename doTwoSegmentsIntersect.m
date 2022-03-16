function [doSegmentsIntersect,intersection] = doTwoSegmentsIntersect(p1,p2,p3,p4)
% input: two segments described by their respective vertices p1, p2 and p3, 
% p4 
% output: true(1) or false(0), and if the answer is true(1), the 
% intersection point

doSegmentsIntersect = false;
intersection = 'line segments do not intersect';
sa_num = (((p1(1)-p3(1))*(p3(2)-p4(2)))-((p1(2)-p3(2))*(p3(1)-p4(1))));
sa_denom = (((p1(1)-p2(1))*(p3(2)-p4(2)))-((p1(2)-p2(2))*(p3(1)-p4(1))));
sa = sa_num/sa_denom;
sb = (((p2(1)-p1(1))*(p1(2)-p3(2)))-((p2(2)-p1(2))*(p1(1)-p3(1))))/sa_denom;

% sort points st p1<p2, p3<p4, p1<=p3
points = [sort([p1;p2]);sort([p3;p4])];
if (points(1,1) > points(3,1)) || (points(1,2) > points(3,2))
    points = [points(3:4,:);points(1:2,:)];
end

% if line segments are colinear. may intersect in a point or segment.
if sa_num == 0 && sa_denom == 0
    % if intersection is a point:
    if points(2,:) == points(3,:)
        doSegmentsIntersect = true;
        intersection = points(2,:);
        return
    % if intersection is a segment:
    elseif (points(2,1) > points(3,1)) || (points(2,2) > points(3,2))
        doSegmentsIntersect = true;
        intersection = [max(points(1,:),points(3,:));min(points(2,:),points(4,:))];
        return
    end
% if line segments are not parallel. may intersect at a point. 
elseif sa_denom ~= 0
    % Check if intersection point actually belongs to segments
    if sa >= 0 && sa <= 1 && sb >= 0 && sb <= 1
        doSegmentsIntersect = true;
        intersection = [p1(1)+sa*(p2(1)-p1(1)), p1(2)+sa*(p2(2)-p1(2))];
        return
    end
end
end

