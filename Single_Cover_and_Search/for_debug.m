clc;clear;close all
load('pos_fine.mat');

% 栅格地图尺寸
velocity_max = 5.14444;  % 最大航行速度：10节
size_coverage = 150;  % 探查范围：150m
gdata=rgb2gray(imread('Maps/mymap2.bmp'));  % 从bmp文件读取地图（4*3海里）
mapOriginal=imbinarize(gdata);  % 二维化

% 路径规划参数
size_shange = 300;
fine_step = 30;   % 精细路径的步长

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

resolutionX=size(mapOriginal,1);
resolutionY=size(mapOriginal,2);
map_tosearch = mapOriginal;

size_line=2;%线宽
tt=0:pi/500:2*pi;
route_length = 0;  % 路径总长（单位m）

f3=figure('Name','Route.','Color','white');
f3.Position = [0 50 1000 750];
image((mapOriginal==0).*170 + (mapOriginal==1).*70);
colormap turbo
image((mapOriginal==0).*128 + (mapOriginal==1).*250);
colormap(gray(256))

hold on
axis([0 resolutionY 0 resolutionX])%限制图的边界
plot(pos_fine(1,1),pos_fine(1,2), 'o','markersize', 10,'markerfacecolor','k','MarkerEdgeColor', 'k')
hd1=line('Color','k','Marker','o', 'MarkerSize',10);%用来设置运动点颜色/形状/大小
for i=2:size(pos_fine,1)
    pos_last = [pos_fine(i-1,1),pos_fine(i-1,2)];
    pos_now = [pos_fine(i,1),pos_fine(i,2)];
    plot([pos_last(1) pos_now(1)],[pos_last(2) pos_now(2)],...
        'k','LineWidth',size_line)
    route_length = route_length + norm(pos_now-pos_last);
    set(hd1,'xdata',pos_now(1),'ydata',pos_now(2));  % 用来给运动点设置运动位置的
    
    % 探查范围
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
    drawnow;  % 命令运动点动起来的
end   