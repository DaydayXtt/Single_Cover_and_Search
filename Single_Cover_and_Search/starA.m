function route_re=starA(M,A_num,B_num)
%STARA    根据A*算法计算两点间的最短路径
%   M为矩阵
%   A为起点
%   B为终点，这个终点是一row里的终点？
global F G parent
row = size(M, 1);
col = size(M, 2);
A=[ceil(A_num/col),mod(A_num-1,col)+1];
B=[ceil(B_num/col),mod(B_num-1,col)+1];
notCross = 1; % M中不能通过的点的值
%% ==============================================
F = zeros(size(M));
G = zeros(size(M));
mov = [1 0;-1 0; 0 1;0 -1;-1 -1; -1 1; 1 -1; 1 1];  % 八种动作
%mov = [1 0;-1 0; 0 1;0 -1];
opened= [A];
closed = [];
parent = zeros(size(M));
inOpen = false(size(M));   % 是否在开放列表中
inClose = false(size(M));  % 是否在关闭列表中
%% ================================================
%
inOpen(A(1),A(2)) = true;
G(A(1),A(2)) = 0;
F(A(1),A(2)) = hn(A,B);  % A、B两点的曼哈顿距离（横纵坐标差的绝对值之和）
[index,min] = minInOpen(opened);  % open列表中fn最小的节点序号和行列位置
while min(1)~=B(1)||min(2)~=B(2)  % 结束条件(也就是终点被放入到open)
    opened(index,:) = [];  % 从开放列表中删除
    inOpen(min(1), min(2)) = false;
    closed = [closed;min];
    inClose(min(1), min(2)) = true;
    
    % 计算8领域中各点的fn，gn，hn。
    for i=1:size(mov,1)
        temp = min + mov(i,:);
        if temp(1)<=row&&temp(1)>0&&temp(2)<=col&&temp(2)>0  % 没出界
            if M(temp(1),temp(2)) ~= notCross && inClose(temp(1),temp(2)) == false
                % 该点可以通过且不在封闭列表中
                if inOpen(temp(1),temp(2)) == 0  % 不在开放列表中，加入open
                    parent(temp(1),temp(2)) = (min(2)-1)*row+ min(1);
                    opened = [opened;temp];
                    G(temp(1),temp(2)) = gn(temp,row);
                    F(temp(1),temp(2)) = G(temp(1),temp(2))+hn(temp,B);
                    inOpen(temp(1), temp(2)) = true;
                else           % 在开放列表中
                    gnn = norm(min-temp) + G(min(1),min(2)); %
                    if gnn < G(temp(1),temp(2))
                        parent(temp(1),temp(2)) = downRank(min,row);
                    end
                end
            end
        end
    end
    
    [index,min] = minInOpen(opened);
end
% 回溯找路径
route =[B];
t=parent(B(1),B(2));
indA = downRank(A,row);
while t ~=  indA
    pc = upRank2(t,row);
    route = [route;pc];
    t=parent(t);
end
route = [route;A];
route=flipud(route);
route(:,3)=(route(:,1)-1)*col+route(:,2);
route_re=route(:,3)';
