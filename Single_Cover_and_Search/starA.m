function route_re=starA(M,A_num,B_num)
%STARA    ����A*�㷨�������������·��
%   MΪ����
%   AΪ���
%   BΪ�յ㣬����յ���һrow����յ㣿
global F G parent
row = size(M, 1);
col = size(M, 2);
A=[ceil(A_num/col),mod(A_num-1,col)+1];
B=[ceil(B_num/col),mod(B_num-1,col)+1];
notCross = 1; % M�в���ͨ���ĵ��ֵ
%% ==============================================
F = zeros(size(M));
G = zeros(size(M));
mov = [1 0;-1 0; 0 1;0 -1;-1 -1; -1 1; 1 -1; 1 1];  % ���ֶ���
%mov = [1 0;-1 0; 0 1;0 -1];
opened= [A];
closed = [];
parent = zeros(size(M));
inOpen = false(size(M));   % �Ƿ��ڿ����б���
inClose = false(size(M));  % �Ƿ��ڹر��б���
%% ================================================
%
inOpen(A(1),A(2)) = true;
G(A(1),A(2)) = 0;
F(A(1),A(2)) = hn(A,B);  % A��B����������پ��루���������ľ���ֵ֮�ͣ�
[index,min] = minInOpen(opened);  % open�б���fn��С�Ľڵ���ź�����λ��
while min(1)~=B(1)||min(2)~=B(2)  % ��������(Ҳ�����յ㱻���뵽open)
    opened(index,:) = [];  % �ӿ����б���ɾ��
    inOpen(min(1), min(2)) = false;
    closed = [closed;min];
    inClose(min(1), min(2)) = true;
    
    % ����8�����и����fn��gn��hn��
    for i=1:size(mov,1)
        temp = min + mov(i,:);
        if temp(1)<=row&&temp(1)>0&&temp(2)<=col&&temp(2)>0  % û����
            if M(temp(1),temp(2)) ~= notCross && inClose(temp(1),temp(2)) == false
                % �õ����ͨ���Ҳ��ڷ���б���
                if inOpen(temp(1),temp(2)) == 0  % ���ڿ����б��У�����open
                    parent(temp(1),temp(2)) = (min(2)-1)*row+ min(1);
                    opened = [opened;temp];
                    G(temp(1),temp(2)) = gn(temp,row);
                    F(temp(1),temp(2)) = G(temp(1),temp(2))+hn(temp,B);
                    inOpen(temp(1), temp(2)) = true;
                else           % �ڿ����б���
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
% ������·��
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
