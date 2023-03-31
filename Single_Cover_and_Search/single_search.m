% ������������򸲸�+Ŀ��̽��
% ���ڼ��裺�����ͼ��Ϣ��֪��һ�����뻷��
% ·��������դ�񷨣�֮��ת����λ�����겢ϸ��(pos_fine)��̽��Ŀ��
% ������û��100%�����Կ��Ǹ��ݶ�ά����ͼ��ϢmapOriginal�޸�դ���ͼG
% num_available = 100;  % ����Ŀ���ܴ�̽����������ֱֵ��Ӱ��ָ��2.
% ��������ʱ��϶�̫������һ����4��
clc;clear;close all
tStart = tic;
%% ׼��
% դ���ͼ�ߴ�
velocity_max = 5.14444;  % ������ٶȣ�10��=5.14444 m/s
size_coverage = 150;  % ̽�鷶Χ��150��
gdata=rgb2gray(imread('Maps/mymap2.png'));  % ��bmp�ļ���ȡ��ͼ��4*3���
mapOriginal=imbinarize(gdata);  % ��ά��
size_shange = 300;
% ��չ��ͼ��Ե
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

% ��Ŀ�����
rng(10);
dist_find = 10;  % �ֽ�һ������(10m)�����Ŀ��λ�ñ��
tolerance = 1;  % �ɹ�̽��ı�׼����ǵ�Ŀ��λ����ʵ��������1m����
num_detect = 20;  % ��Ŀ�����
num_available = 100;  % ����Ŀ���ܴ�̽�����
dist_threat = 5;  % Ŀ����в����뾶(5m)���������һ��Эͬʧ��
pos_detect = zeros(num_detect,7);number = 1;  % λ�����꣬�͸���̽������
pos_label = zeros(num_detect,2);  % ���Ŀ��

resolutionX=size(mapOriginal,1);
resolutionY=size(mapOriginal,2);
map_tosearch = mapOriginal;  % �������򸲸����õ�
fine_step = 15;   % ��ϸ·��pos_fine�Ĳ���
coordinate_fail = 0;  % Эͬʧ�ܴ���

% �������num_detect����Ŀ��
while number ~= num_detect+1
    % ��Ŀ��ĸ��ֲ�����[������ ������ �Ƿ񱻱��(1/0) �ܴ�̽����� ̽��ִ�д��� ��ȷ̽����� ��в����]
    pos_d = [ceil(resolutionY*rand),ceil(resolutionX*rand),1,num_available,0,0,dist_threat];
    if mapOriginal(pos_d(2),pos_d(1)) == 1  % Ŀ���ڲ����ߵĵط�������
        pos_detect(number,:) = pos_d;
        number = number + 1;
    end
end
%% ·���滮
route=[];
for i=1:shange_row
    index_num=(i-1)*shange_col+1:(i-1)*shange_col+shange_col;  % λ�����
    index1=find(G(i,:)==0);
    if ~isempty(index1)
        index_num=index_num(index1);  % �ҵ���i��"0"(���ϰ���)��λ��
        if mod(i,2)==1  % �������У���������
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
                if index_num(j)-index_num(j-1) == 1  % ����������������
                    flag=1;
                end
            end
            if flag==1
                E=index_num(end);
                route_lin=starA(G,S,E);  % �ҵ�S��E��·��
                route=[route route_lin];
                S=E;  % �������
            end
        else  % ż���У���������
            index_num=fliplr(index_num);  % ˮƽ��ת
            if i==2
                S=index_num(1);
                S_start = S;
            else
                E=index_num(1);  % �յ㣬��һ�е�ĩβ
                route_lin=starA(G,S,E);
                route=[route route_lin];
                S=E;  % �������
            end
            for j=2:length(index_num)
                flag=0;
                if index_num(j)-index_num(j-1)~=-1  % ������
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
                if index_num(j)-index_num(j-1)==-1  % ����
                    flag=1;
                end
            end
            if flag==1
                E=index_num(end);
                route_lin=starA(G,S,E);
                route=[route route_lin];
                S=E;  % �������
            end
        end
    end
end
for i=length(route):-1:2  % ɾ���յ㴦�ظ���·��
    if route(i)==route(i-1)
        route(i)=[];
    end
end

% դ��·����Ϣת��Ϊλ�����겢ϸ��
position = [(mod(route-1,shange_col)+0.5)*size_shange;...
    (ceil(route/shange_col)-0.5)*size_shange]';
pos_fine = [];  % ����ϸ��·����Ϣ
for i = 2:size(position,1)
    xq = linspace(position(i-1,1),position(i,1),size_coverage/fine_step+1);
    yq = linspace(position(i-1,2),position(i,2),size_coverage/fine_step+1);
    xyq = [xq' yq'];
    pos_fine = [pos_fine; xyq];
end
for i=length(pos_fine):-1:2  % ɾ���յ㴦�ظ���·��
    if pos_fine(i,:)==pos_fine(i-1,:)
        pos_fine(i,:)=[];
    end
end

%% ����·��pos_fine��̽��Ŀ��
for i = 1:size(pos_fine,1)
    dist = pos_detect(:,1:2) - pos_fine(i,:);
    dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % ����������������Ŀ���ľ���
    if min(dist) <= size_coverage  % �������ߵ���һ����Ŀ����̽�鷶Χ��
        [value,id]=min(dist);
        if pos_detect(id,3) == 1  % �õ�û����ǣ���ʼ̽��
            pos_detect(id,4) = max(0,pos_detect(id,4)-1);  % ʣ��̽�����-1
            pos_detect(id,5) = pos_detect(id,5)+1;  % ִ��̽�����+1
            nearest_dest = pos_detect(id,1:2) - pos_fine(i,:);  % �����嵱ǰλ����̽�鵽�ľ�Ŀ���ľ���
            head_approach = atan2(nearest_dest(2),nearest_dest(1));  % �̶��ĵֽ��Ƕ�
            pos_find = pos_fine(i,:) + (value-dist_find)*[cos(head_approach),sin(head_approach)];  % ̽�鵽��Ŀ���ʱ���������λ��
            dist_detect = norm(pos_find-pos_detect(id,1:2));  % ̽�鵽��Ŀ���ʱ���������Ŀ��ľ��루��ôŪ����10m��
            pos_label(id,:) = pos_find + dist_detect*[cos(head_approach),sin(head_approach)];  % ��Ǿ�Ŀ�������
            if norm(pos_label(id,:) - pos_detect(id,1:2)) <= tolerance
                pos_detect(id,3) = 0;  % ��Ŀ����Ѿ���̽�鵽�����
                pos_detect(id,6) = pos_detect(id,6)+1;  % ��ȷ̽�����+1
            end

            % ̽�鵽֮�󣬴ӵ�ǰλ��ʻ�������ԭ·��pos_fine
            dist = pos_find - pos_fine(i:i+floor(900/fine_step),:);
            dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % ��������������������ɸ�·����ľ���
            [~,idd]=min(dist);
            pos_return = pos_fine(i+idd-1,:);
            pos_fine(i+1:i+idd-1,:) = 0;
            pos_fine(i+1,:) = pos_find;  % ̽��Ŀ�꣬����·����Ҫ����
            pos_fine(i+idd-1,:) = pos_return;

            % ����Ƿ�������в����
            xq = linspace(pos_fine(i+1,1),pos_fine(i+idd-1,1),size_coverage/1+1);
            yq = linspace(pos_fine(i+1,2),pos_fine(i+idd-1,2),size_coverage/1+1);
            xyq = [xq' yq'];
            dist = pos_detect(id,1:2) - xyq;
            dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % ��������������������ɸ�·����ľ���
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
for i=length(pos_fine):-1:2  % ɾ����̽������ߵ�·��
    if pos_fine(i,:)==0
        pos_fine(i,:)=[];
    end
end
%% ���ӻ�
f3=figure('Name','Route.','Color','white');
f3.Position = [0 50 1000 750];
title('������Ϣ�ɼ������ʾ')
xlabel('X(��)')
ylabel('Y(��)')
hold on
axis([0 resolutionY 0 resolutionX])

% 1����ͼ
image((mapOriginal==0).*128 + (mapOriginal==1).*255);
colormap(gray(256))

% 2����Ŀ���
tt=0:pi/500:2*pi;
for i = 1:num_detect
    p0 = plot(pos_detect(i,1),pos_detect(i,2),...
        '.','markersize', 20,'markerfacecolor','b','MarkerEdgeColor', 'b',...
        'DisplayName','��Ŀ�꣨��̽�飩');
    xx=pos_detect(i,1)+size_coverage*sin(tt);
    yy=pos_detect(i,2)+size_coverage*cos(tt);
    xx=round(xx);yy=round(yy);
    p1 = fill(xx,yy,'b','facealpha',0.2,'LineStyle','none',...
        'DisplayName','̽�鷶Χ����̽�飩');
end

% 3����㡢�յ�
Sx=pos_fine(1,1);Sy=pos_fine(1,2);  % ���
Ex=pos_fine(end,1);Ey=pos_fine(end,2);  % �յ�
p2 = plot(Sx,Sy, 'o','markersize', 10,'markerfacecolor','r','MarkerEdgeColor', 'r',...
    'DisplayName','��㡢�յ�');
plot(Ex,Ey, 'o','markersize', 10,'markerfacecolor','r','MarkerEdgeColor', 'r')
set(gca,'YDir','reverse');  % Y�᷽��ת
% �������򸲸Ƿ�Χ
xx=pos_fine(1,1)+size_coverage*sin(tt);
yy=pos_fine(1,2)+size_coverage*cos(tt);
xx=round(xx);yy=round(yy);
p3 = fill(xx,yy,'k','facealpha',0.1,'LineStyle','none',...
    'DisplayName','���Ƿ�Χ');
size_line=2;  % �߿�
for ii = min(xx):max(xx)  % ���㸲������õ�
    for jj = min(yy):max(yy)
        if norm([ii jj] - pos_fine(1,:)) <= size_coverage
            map_tosearch(jj,ii) = 0;
        end
    end
end

% 4��·��
hd1=line('Color','r','Marker','o', 'MarkerSize',10);  % �����˶�����ɫ/��״/��С
route_length = 0;  % ·���ܳ�����λm��
for i=2:size(pos_fine,1)
    pos_last = [pos_fine(i-1,1),pos_fine(i-1,2)];
    pos_now = [pos_fine(i,1),pos_fine(i,2)];
    p4 = plot([pos_last(1) pos_now(1)],[pos_last(2) pos_now(2)],...
        'r','LineWidth',size_line,...
        'DisplayName','����·��');
    route_length = route_length + norm(pos_now-pos_last);  % ·���������
    
    % ���򸲸Ƿ�Χ(150m)
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
    dist = sqrt(dist(:,1).^2+dist(:,2).^2);  % ����������������Ŀ��������ľ���
    [value_1,idddd]=min(dist);
    if round(value_1) <= round(dist_detect)
        p0 = plot(pos_label(idddd,1), pos_label(idddd,2),...
            '.','markersize',20,'markerfacecolor','g','MarkerEdgeColor','g',...
            'DisplayName','��Ŀ��㣨�ɹ�̽�飩');
        xx=pos_label(idddd,1)+size_coverage*sin(tt);
        yy=pos_label(idddd,2)+size_coverage*cos(tt);
        xx=round(xx);yy=round(yy);
        p1 = fill(xx,yy,'g','facealpha',0.2,'LineStyle','none',...
            'DisplayName','̽�鷶Χ���ɹ�̽�飩');
        text(pos_label(idddd,1)+30, pos_label(idddd,2)-20,...
            ['Ŀ��' num2str(idddd) '��(' num2str(pos_label(idddd,1)+5) ',' num2str(pos_label(idddd,2)+5),').'],...
            'FontSize', 10);
    end
        
    set(hd1,'xdata',pos_now(1),'ydata',pos_now(2),...
        'DisplayName','�����嵱ǰλ��');  % �������˶��������˶�λ�õ�
%     legend([p0 p1 p2 p3 p4 hd1]);drawnow;  % ��̬
end
legend([p0 p1 p2 p3 p4]);
%% ���ǽ��ʾ��
% f4=figure('Name','Coverage.','Color','white');
% f4.Position = [0 50 1000 750];image((map_tosearch==0).*128 + (map_tosearch==1).*255);
% colormap(gray(256));hold on
% axis([0 resolutionY 0 resolutionX])  % ����ͼ�ı߽�
%% ָ��
% save('pos_fine.mat','pos_fine');
target1 = round(100-sum(map_tosearch(:))/sum(mapOriginal(:))*100);
target2 = round(sum(pos_detect(:,4))/(num_available*num_detect)*100);
target3 = round(sum(pos_detect(:,6))/sum(pos_detect(:,5))*100);
target4 = round(route_length/velocity_max/60);
target5 = round(coordinate_fail);

% ����͹۷�
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
disp(['����·���ܳ��ȣ�',num2str(route_length),'��'])
disp(['1��������Ч�����ʣ�',num2str(target1),'%.'])
disp(['2����Ŀ����Ч̽���ʣ�',num2str(target2),'%'])
disp(['3����Ŀ����ȷ̽���ʣ�',num2str(target3),'%'])
disp(['4��ȫ������Ч��������ʱ����',num2str(target4),'����'])
disp(['5��Эͬʧ���ܴ�����',num2str(target5),'��'])
disp(['�͹۵÷֣�',num2str(score1),'+',num2str(score2),'+',...
    num2str(score3),'+',num2str(score4),'+',num2str(score5),...
    '=',num2str(score),'��']);
disp('--------------------------------------------')
tEnd = toc(tStart);
disp(['�����ʱ��',num2str(tEnd),'��']);