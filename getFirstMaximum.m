function [xPoint] = getFirstMaximum(x, y, k, m)
%�õ�ʹy�õ���һ������ֵ��xֵ
%���������������ڵ����������������б�ʾ���ֵ����k
xPoint = -1;
min = y(1);
    for i = 2:1:length(x)-1
        if abs(y(i-1)) > 1
            continue;
        end
        y1 = y(i) - y(i+1);
        y2 = y(i) - y(i-1);
        if y2 < 0
            min = y(i);
        end
        if y1 > 0 && y2 > 0
            x1 = x(i) - x(i+1);
            x2 = x(i) - x(i-1);
            if x1 > 0 || x2 < 0
                disp('error');
                continue;
            end
            if abs(y1/x1) > k && abs(y2/x2) > k && y(i) - min > m
                xPoint = x(i);
                return;
            end
        end
    end
end