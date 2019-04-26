function [plArr] = draw3DCamera(T_world_cam, size, color)
%DRAW3DCAMERA renders a 3D camera with lines.

R_w_c = T_world_cam(1:3, 1:3);
t_w_c = T_world_cam(1:3, 4);

% pt1: the start point of each line. (in cam frame)
% pt2: the end point of each line. (in cam frame)
pt1 = size*[0, 0, 0, 0, 1, -1, -1, 1;
       0, 0, 0, 0, 1, 1, -1, -1;
       0, 0, 0, 0, 1, 1, 1, 1];
   
pt2 = size*[1, -1, -1, 1, -1, -1, 1, 1;
       1, 1, -1, -1, 1, -1, -1, 1;
       1, 1, 1, 1, 1, 1, 1, 1];
   
tArr = ones(3, length(pt1));
tArr(1, :) = tArr(1, :).*t_w_c(1);
tArr(2, :) = tArr(2, :).*t_w_c(2);
tArr(3, :) = tArr(3, :).*t_w_c(3);
   
pt1 = R_w_c*pt1 + tArr;
pt2 = R_w_c*pt2 + tArr;


plArr = [];

for idx = (1:length(pt1))
    plArr = [plArr, line([pt1(1, idx), pt2(1, idx)], [pt1(2, idx), pt2(2, idx)], [pt1(3, idx), pt2(3, idx)], 'Color', color)];
end
   
end

