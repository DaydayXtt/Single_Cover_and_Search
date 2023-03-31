function [v] = downRank(point,row)
%从二维坐标变到一维
v = (point(2)-1)*row+ point(1);
end