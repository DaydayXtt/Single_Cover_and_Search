% 单智能体的区域覆盖+目标探查
% 基于假设：整体地图信息已知、一切理想环境
% 路径覆盖用栅格法，之后转换成位置坐标并细化(pos_fine)再探查目标
% 覆盖率没有100%，可以考虑根据二维化地图信息mapOriginal修改栅格地图G
% num_available = 100;  % 单个目标总带探查次数，这个值直接影响指标2.
% 单智能体时间肯定太长，下一步试4个
clc;clear;close all
tStart = tic;
%% 准备
% 栅格地图尺寸
velocity_max = 5.14444;  % 最大航行速度：10节=5.14444 m/s
size_coverage = 150;  % 探查范围：150米
gdata=rgb2gray(imread('Maps/mymap2.png'));  % 从bmp文件读取地图（4*3海里）
mapOriginal=imbinarize(gdata);  % 二维化
size_shange = 300;
% 扩展地图边缘
x_ex = ceil(size(mapOriginal,2)/size_shange)*size_shange;
y_ex = ceil(size(mapOriginal,1)/size_shange)*size_shange;
mapOriginal = [mapOriginal ...
    imbinarize(zeros(size(mapOriginal,1),x_ex-size(mapOriginal,2)))];
mapOriginal = [mapOriginal;...
    imbinarize(zeros(y_ex-size(mapOriginal,1),x_ex))];
shange_row = floor(size(mapOriginal,1)/size_shange);
shange_col = floor(size(mapOriginal,2)/size_shange);
G = imresize(mapOriginal,[shange_row shange_col]);
G = ~G;

% 静目标参数
rng(10);
dist_find = 10;  % 抵近一定距离(10m)后进行目标位置标记
tolerance = 1;  % 成功探查的标准：标记的目标位置与实际误差距离1m以内
num_detect = 20;  % 静目标个数
num_available = 100;  % 单个目标总带探查次数
dist_threat = 5;  % 目标威胁区域半径(5m)，进入则记一次协同失败
pos_detect = zeros(num_detect,7);number = 1;  % 位置坐标，和各种探查属性
pos_label = zeros(num_detect,2);  % 标记目标

resolutionX=size(mapOriginal,1);
resolutionY=size(mapOriginal,2);
map_tosearch = mapOriginal;  % 计算区域覆盖率用的
fine_step = 15;   % 精细路径pos_fine的步长
coordinate_fail = 0;  % 协同失败次数

% 随机生成num_detect个静目标
while number ~= num_detect+1
    % 静目标的各种参数：[横坐标 纵坐标 是否被标记(1/0) 总带探查次数 探查执行次数 正确探查次数 威胁区域]
    pos_d = [ceil(resolutionY*rand),ceil(resolutionX*rand),1,num_available,0,0,dist_threat];
    if mapOriginal(pos_d(2),pos_d(1)) == 1  % 目标在不能走的地方，舍弃
        pos_detect(number,:) = pos_d;
        number = number + 1;
    end
end
%% 路径规划
route=[];
for i=1:shange_row
    index_num=(i-1)*shange_col+1:(i-1)*shange_col+shange_col;  % 位置序号
    index1=find(G(i,:)==0);
    if ~isempty(index1)
        index_num=index_num(index1);  % 找到第i行"0"(非障碍物)的位置
        if mod(i,2)==1  % 是奇数行，从左往右
            if i==1
                S=index_num(1);
                S_start = S;
            else
                E=index_num(1);
                route_lin=starA(G,S,E);
                route=[route route_lin];
                S=E;
            end
            for j=2:length(index_num)
                flag=0;
                if index_num(j)-index_num(j-1)~=1
                    if flag==1
                        E=index_num(j-1);
                        route_lin=starA(G,S,E);
                        route=[route route_lin];
                        S=E;
                        E=index_num(j);
                        route_lin=starA(G,S,E);
                        route=[route route_lin];
                        S=E;
                    else
                        E=index_num(j);
                        route_lin=starA(G,S,E);
                        route=[route route_lin];
                        S=E;
                        flag=1;
                    end
                end
                if index_num(j)-index_num(j-1) == 1  % 连着两个都可以走
                    flag=1;
                end
            end
            if flag==1
                E=index_num(end);
                route_lin=starA(G,S,E);  % 找到S到E的路径
                route=[route route_lin];
                S=E;  % 更新起点
            end
        else  % 偶数行，从右往左
            index_num=fliplr(index_num);  % 水平翻转
            if i==2
                S=index_num(1);
                S_start = S;
            else
                E=index_num(1);  % 终点，这一行的末尾
                route_lin=starA(G,S,E);
                route=[route route_lin];
                S=E;  % 更新起点
            end
            for j=2:length(index_num)
                flag=0;
                if index_num(j)-index_num(j-1)~=-1  % 不连着
                    if flag==1
                        E=index_num(j-1);
                        route_lin=starA(G,S,E);
                        route=[route route_lin];
                        S=E;
                        E=index_num(j);
                        route_lin=starA(G,S,E);
                        route=[route route_lin];
                        S=E;
                    else
                        E=index_num(j);
                        route_lin=starA(G,S,E);
                        route=[route route_lin];
                        S=E;
                    end
                end
                if index_num(j)-index_num(j-1)==-1  % 连着
                    flag=1;
                end
            end
            if flag==1
                E=index_num(end);
                route_lin=starA(G,S,E);
                route=[route route_lin];
                S=E;  % 更新起点
            end
        end
    end
end
for i=length(route):-1:2  % 删掉拐点处重复的路径
    if route(i)==route(i-1)
        route(i)=[];
    end
end

% 栅格路径信息转换为位置坐标并细化
position = [(mod(route-1,shange_col)+0.5)*size_shange;...
    (ceil(route/shange_col)-0.5)*size_shange]';
pos_fine = [];  % 更精细的路径信息
for i = 2:size(position,1)
    xq = linspace(position(i-1,1),position(i,1),size_coverage/fine_step+1);
    yq = linspace(position(i-1,2),position(i,2),size_coverage/fine_step+1);
    xyq = [xq' yq'];
    pos_fine = [pos_fine; xyq];
end
for i=length(pos_fine):-1:2  % 删掉拐点处重复的路径
    if pos_fine(i,:)==pos_fine(i-1,:)
        pos_fine(i,:)=[];
    end
end

%% 遍历路径pos_fine，探查目标
for i = 1:size(pos_fine,1)
    dist = pos_detect(:,1:2) - pos_fine(i,:);
    dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % 计算智能体距离各个目标点的距离
    if min(dist) <= size_coverage  % 智能体走到了一个静目标点的探查范围内
        [value,id]=min(dist);
        if pos_detect(id,3) == 1  % 该点没被标记，开始探查
            pos_detect(id,4) = max(0,pos_detect(id,4)-1);  % 剩余探查次数-1
            pos_detect(id,5) = pos_detect(id,5)+1;  % 执行探查次数+1
            nearest_dest = pos_detect(id,1:2) - pos_fine(i,:);  % 智能体当前位置与探查到的静目标点的距离
            head_approach = atan2(nearest_dest(2),nearest_dest(1));  % 固定的抵近角度
            pos_find = pos_fine(i,:) + (value-dist_find)*[cos(head_approach),sin(head_approach)];  % 探查到静目标点时，智能体的位置
            dist_detect = norm(pos_find-pos_detect(id,1:2));  % 探查到静目标点时智能体距离目标的距离（这么弄就是10m）
            pos_label(id,:) = pos_find + dist_detect*[cos(head_approach),sin(head_approach)];  % 标记静目标点坐标
            if norm(pos_label(id,:) - pos_detect(id,1:2)) <= tolerance
                pos_detect(id,3) = 0;  % 该目标点已经被探查到，标记
                pos_detect(id,6) = pos_detect(id,6)+1;  % 正确探查次数+1
            end

            % 探查到之后，从当前位置驶向最近的原路径pos_fine
            dist = pos_find - pos_fine(i:i+floor(900/fine_step),:);
            dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % 计算智能体距离往后若干个路径点的距离
            [~,idd]=min(dist);
            pos_return = pos_fine(i+idd-1,:);
            pos_fine(i+1:i+idd-1,:) = 0;
            pos_fine(i+1,:) = pos_find;  % 探查目标，覆盖路径需要更新
            pos_fine(i+idd-1,:) = pos_return;

            % 检查是否落入威胁区域
            xq = linspace(pos_fine(i+1,1),pos_fine(i+idd-1,1),size_coverage/1+1);
            yq = linspace(pos_fine(i+1,2),pos_fine(i+idd-1,2),size_coverage/1+1);
            xyq = [xq' yq'];
            dist = pos_detect(id,1:2) - xyq;
            dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % 计算智能体距离往后若干个路径点的距离
            [value_1,iddd]=min(dist);
            if value_1 <= dist_threat
                coordinate_fail = coordinate_fail + 1;
            end
%             plot(pos_find(1),pos_find(2),'pk','LineWidth',size_line)
%             plot(pos_return(1),pos_return(2),'.k','LineWidth',size_line)
%             plot([pos_fine(i,1) pos_find(1)],[pos_fine(i,2) pos_find(2)],...
%                 ':r','LineWidth',size_line)
%             plot([pos_find(1) pos_return(1)],[pos_find(2) pos_return(2)],...
%                 ':r','LineWidth',size_line)
        end
    end
end
for i=length(pos_fine):-1:2  % 删掉因探查而不走的路径
    if pos_fine(i,:)==0
        pos_fine(i,:)=[];
    end
end
%% 可视化
f3=figure('Name','Route.','Color','white');
f3.Position = [0 50 1000 750];
title('区域信息采集结果演示')
xlabel('X(米)')
ylabel('Y(米)')
hold on
axis([0 resolutionY 0 resolutionX])

% 1、地图
image((mapOriginal==0).*128 + (mapOriginal==1).*255);
colormap(gray(256))

% 2、静目标点
tt=0:pi/500:2*pi;
for i = 1:num_detect
    p0 = plot(pos_detect(i,1),pos_detect(i,2),...
        '.','markersize', 20,'markerfacecolor','b','MarkerEdgeColor', 'b',...
        'DisplayName','静目标（待探查）');
    xx=pos_detect(i,1)+size_coverage*sin(tt);
    yy=pos_detect(i,2)+size_coverage*cos(tt);
    xx=round(xx);yy=round(yy);
    p1 = fill(xx,yy,'b','facealpha',0.2,'LineStyle','none',...
        'DisplayName','探查范围（待探查）');
end

% 3、起点、终点
Sx=pos_fine(1,1);Sy=pos_fine(1,2);  % 起点
Ex=pos_fine(end,1);Ey=pos_fine(end,2);  % 终点
p2 = plot(Sx,Sy, 'o','markersize', 10,'markerfacecolor','r','MarkerEdgeColor', 'r',...
    'DisplayName','起点、终点');
plot(Ex,Ey, 'o','markersize', 10,'markerfacecolor','r','MarkerEdgeColor', 'r')
set(gca,'YDir','reverse');  % Y轴方向翻转
% 起点的区域覆盖范围
xx=pos_fine(1,1)+size_coverage*sin(tt);
yy=pos_fine(1,2)+size_coverage*cos(tt);
xx=round(xx);yy=round(yy);
p3 = fill(xx,yy,'k','facealpha',0.1,'LineStyle','none',...
    'DisplayName','覆盖范围');
size_line=2;  % 线宽
for ii = min(xx):max(xx)  % 计算覆盖面积用的
    for jj = min(yy):max(yy)
        if norm([ii jj] - pos_fine(1,:)) <= size_coverage
            map_tosearch(jj,ii) = 0;
        end
    end
end

% 4、路径
hd1=line('Color','r','Marker','o', 'MarkerSize',10);  % 设置运动点颜色/形状/大小
route_length = 0;  % 路径总长（单位m）
for i=2:size(pos_fine,1)
    pos_last = [pos_fine(i-1,1),pos_fine(i-1,2)];
    pos_now = [pos_fine(i,1),pos_fine(i,2)];
    p4 = plot([pos_last(1) pos_now(1)],[pos_last(2) pos_now(2)],...
        'r','LineWidth',size_line,...
        'DisplayName','航行路径');
    route_length = route_length + norm(pos_now-pos_last);  % 路径长度求和
    
    % 区域覆盖范围(150m)
    xx=pos_now(1)+size_coverage*sin(tt);
    yy=pos_now(2)+size_coverage*cos(tt);
    xx=round(xx);yy=round(yy);
    for ii = max(1,min(xx)):min(resolutionY,max(xx))
        for jj = max(1,min(yy)):min(resolutionY,max(yy))
            if norm([ii jj] - pos_now) <= size_coverage
                map_tosearch(jj,ii) = 0;
            end
        end
    end
    fill(xx,yy,'k','facealpha',0.05,'LineStyle','none');
% legend([p0 p1 p2 p3]);
    dist = pos_fine(i,:) - pos_detect(:,1:2);
    dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % 计算智能体距离各个目标标记坐标的距离
    [value_1,idddd]=min(dist);
    if round(value_1) <= round(dist_detect)
        p0 = plot(pos_label(idddd,1), pos_label(idddd,2),...
            '.','markersize',20,'markerfacecolor','g','MarkerEdgeColor','g',...
            'DisplayName','静目标点（成功探查）');
        xx=pos_label(idddd,1)+size_coverage*sin(tt);
        yy=pos_label(idddd,2)+size_coverage*cos(tt);
        xx=round(xx);yy=round(yy);
        p1 = fill(xx,yy,'g','facealpha',0.2,'LineStyle','none',...
            'DisplayName','探查范围（成功探查）');
        text(pos_label(idddd,1)+30, pos_label(idddd,2)-20,...
            ['目标' num2str(idddd) '：(' num2str(pos_label(idddd,1)+5) ',' num2str(pos_label(idddd,2)+5),').'],...
            'FontSize', 10);
    end
        
    set(hd1,'xdata',pos_now(1),'ydata',pos_now(2),...
        'DisplayName','智能体当前位置');  % 用来给运动点设置运动位置的
%     legend([p0 p1 p2 p3 p4 hd1]);drawnow;  % 动态
end
legend([p0 p1 p2 p3 p4]);
%% 覆盖结果示意
% f4=figure('Name','Coverage.','Color','white');
% f4.Position = [0 50 1000 750];image((map_tosearch==0).*128 + (map_tosearch==1).*255);
% colormap(gray(256));hold on
% axis([0 resolutionY 0 resolutionX])  % 限制图的边界
%% 指标
% save('pos_fine.mat','pos_fine');
target1 = round(100-sum(map_tosearch(:))/sum(mapOriginal(:))*100);
target2 = round(sum(pos_detect(:,4))/(num_available*num_detect)*100);
target3 = round(sum(pos_detect(:,6))/sum(pos_detect(:,5))*100);
target4 = round(route_length/velocity_max/60);
target5 = round(coordinate_fail);

% 计算客观分
if target1>=0 && target1<=80,score1 = 3*target1/80;
elseif target1>80 && target1<=100,score1 = 7*target1/20-25;
else,score1 = 0;
end
if target2>=0 && target2<=80,score2 = 6*target2/80;
elseif target2>80 && target2<=100,score2 = 4*target2/20-10;
else,score2 = 0;
end
score3 = max(0,target3/10);
if target4>=0 && target4<=40,score4 = 10;
elseif target4>40 && target4<=80,score4 = -0.1*target4+14;
else,score4 = 6;
end
if target5>=0 && target5<=5,score5 = -0.8*target5+10;
elseif target5>5 && target5<=10,score5 = -1.2*target5+12;
else,score5 = 0;
end
score = score1 + score2 + score3 + score4 + score5;
%% done
disp('done.')
disp('--------------------------------------------')
disp(['航行路线总长度：',num2str(route_length),'米'])
disp(['1、区域有效覆盖率：',num2str(target1),'%.'])
disp(['2、静目标有效探查率：',num2str(target2),'%'])
disp(['3、静目标正确探查率：',num2str(target3),'%'])
disp(['4、全区域有效覆盖任务时长：',num2str(target4),'分钟'])
disp(['5、协同失败总次数：',num2str(target5),'次'])
disp(['客观得分：',num2str(score1),'+',num2str(score2),'+',...
    num2str(score3),'+',num2str(score4),'+',num2str(score5),...
    '=',num2str(score),'分']);
disp('--------------------------------------------')
tEnd = toc(tStart);
disp(['程序耗时：',num2str(tEnd),'秒']);