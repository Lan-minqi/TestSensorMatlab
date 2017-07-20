function [newx, spline] = continuousInterpolationBySpline(x, y, start, stop, delta)
%对四元数序列进行样条插值，尝试改进
%Reference: Interpolating Three-Dimensional
%Kinematic Data Using Quaternion
%Splines and Hermite Curves
    xindex = 1;
    if x(1) > start
        start = x(1);
    else
        for i = 1:1:length(x)
            if x(i) > start
                xindex = i-1;
                break
            end
        end
    end
    newx = start:delta:stop-delta;
    spline = zeros(4, (stop - start)/delta);
    lasta = 0;
    lastqn = zeros(4,1);
    lastqn1 = zeros(4,1);
    lastt = 0;
    for i = 1:1:length(spline)
        xvalue = start + delta*(i-1);
        if xindex == length(x)
            break;
        end
        if x(xindex+1) < xvalue
            xindex = xindex + 1;
        end
        b = 0;
        c = 0;
        if i == 1
            lasta = Bisect((1+b)*(1-c)*myDouble(y(:,xindex-1), (1-b)*(1+c)*y(:,xindex)),y(:,xindex+1));
            lasta = lasta/norm(lasta);
            lastqn = y(:,xindex);
            lastqn1 = y(:,xindex+1);
            lastt = (xvalue - x(xindex))/(x(xindex+1)-x(xindex));
            continue;
        end
        a = Bisect((1+b)*(1-c)*myDouble(y(:,xindex-1), (1-b)*(1+c)*y(:,xindex)),y(:,xindex+1));
        qb = (1+b)*(1-c)*myDouble(a, lastqn1);
        qb = qb/norm(qb);
        
        p1 = mySlerp(lastqn, lasta, lastt);
        p2 = mySlerp(lasta, qb, lastt);
        p3 = mySlerp(qb, lastqn1, lastt);
        p4 = mySlerp(p1, p2, lastt);
        p5 = mySlerp(p2, p3, lastt);
        spline(:,i-1) = mySlerp(p4,p5,lastt);
        
        spline(:,i-1) = spline(:,i-1)/norm(spline(:,i-1));
        
        lasta = a;
        lasta = lasta/norm(lasta);
        lastqn = y(:,xindex);
        lastqn1 = y(:,xindex+1);        
        lastt = (xvalue - x(xindex))/(x(xindex+1)-x(xindex));
    end
end
function [result] = inverse(a)
    %a为四元数 qw qx qy qz
    result = zeros(4,1);
    x = a(1)^2 + a(2)^2 + a(3)^2 + a(4)^2;
    result(1) = a(1)/x;
    result(2) = -a(2)/x;
    result(3) = -a(3)/x;
    result(4) = -a(4)/x;
end
function [q] = mySlerp(q1, q2, t)
    angle = acos(q1'*q2);
    q = sin((1-t)*angle)*q1/sin(angle) + sin((t)*angle)*q2/sin(angle);
end
function [q] = myDouble(q1, q2)
    q = 2*(q1'*q2)*q2 - q1;
end
function [q] = Bisect(q1, q2)
    q = (q1+q2)/norm(q1+q2);
end