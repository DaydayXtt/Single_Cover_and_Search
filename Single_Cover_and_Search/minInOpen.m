function [index,mc] = minInOpen(open)
%MININOPEN    ����open�б���fn��С�Ľڵ�
%   OPENΪ�����б�
%   
global F
mv =  99999999;
for ii=1:size(open,1)
    v = F(open(ii,1),open(ii,2));
    if v<mv
        mv = v;
        mc = open(ii,:);
        index = ii;
    end
end
end