function [q] = funcSlerp(time1, time2, q1, q2)
    %∂‘time1≤Â÷µ
    q = zeros(4, length(time2));
    j = 1;
    for i = 1:1:length(time2)
        while time1(j) < time2(i) && j < length(time1)
            j = j + 1;
        end
        if j == 1
            q(:,i) = q1(:,1);
        else
            t = (time2(i)-time1(j-1))/(time1(j)-time1(j-1));
            angle = acos(q1(:,j-1)'*q1(:,j));
            q(:,i) = sin((1-t)*angle)*q1(:,j-1)/sin(angle) + sin((t)*angle)*q1(:,j)/sin(angle);
            
        end
    end
end