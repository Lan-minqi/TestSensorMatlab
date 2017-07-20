function [ ] = compareOpPhone(name0, varargin)
    %{
    %name0 手机测量optitrack初始坐标系信息
    %name1 手机信息 
    %name2 Optitrack信息
    %}
    
    [time0, rot0, acc0, mag0, initQ] = readPhoneData(name0, 'absolute');
    if nargin > 1
        [time1, rot1, acc1, mag1, q1] = readPhoneData(varargin{1},'absolute');
        [time2, acc2, q2] = readOpData(varargin{2});
        startTime = time1(1);
        if time1(1) > time2(1)
            startTime = time2(1);
        end
        time1 = time1 - startTime;
        time2 = time2 - startTime;
    end
    q0 = zeros(4,1);
    %q0 optitrack坐标系相对地球坐标系的旋转 qw qx qy qz
    %q1 手机坐标系相对于地球坐标系的旋转 qw qx qy qz
    %q2 刚体坐标系相对于optitrack坐标系的旋转 qx qy qz qw to qw qx qz qy
    %{
    要使得刚体坐标系与手机坐标系对应（轴平行），刚体应该选取为校准方轴，并且在第二次测量之前，校准方轴不能移动

    %}
    point = 5;
    for i = point : 1: length(initQ) - point -1
       q0 = q0 + initQ(:, i);
    end
    q0 = q0 / (length(initQ) - 2*point);
    disp(q0);
    if nargin == 1
        return
    end
    
    temp = q2(1,:);
    q2(1,:) = q2(4,:);
    q2(4,:) = q2(3,:);
    q2(3,:) = q2(2,:);
    q2(2,:) = temp;
    
    %q2 = -q2;
    q3 = inverse(q0);
    q4 = [sqrt(0.5);sqrt(0.5);0;0];
    q5 = [sqrt(0.5);-sqrt(0.5);0;0];
    %q2 = HamiltonProduct(q2,q0);
    
    for i = 1:1:length(q1)
        q1(:,i) = HamiltonProduct(q5 ,HamiltonProduct(q3, HamiltonProduct(q1(:,i),q4)));
        q1(:,i) = q1(:,i)/norm(q1(:,i));
        %disp(norm(q1(:,i)));
        %disp(q1(1,i)^2+q1(2,i)^2+q1(3,i)^2+q1(4,i)^2);
    end
    
    
    
    
    
    
    %q1 = HamiltonProduct(q1, q3);
    %{
    for i = 1:1:length(q1)
        R = zeros(1,9);
        R = getRotationMatrixFromVector(R, q3);
        q1(2:4,i) = matrixMultiVector(R, q1(2:4,i));
        %qtemp = HamiltonProduct(HamiltonProduct(q0, [0;q1(2:4,i)]), q3);
        %q1(2:4,i) = qtemp(2:4);
    end
    %}
    %{
    %旋转角度比较
    angle1 =zeros(1, length(time1));
    for i = 1:1:length(time1)
        angle1(i) = 2*acos(q1(1,i));
    end
    angle2 =zeros(1, length(time2));
    for i = 1:1:length(time2)
        angle2(i) = 2*acos(q2(1,i));
    end
    figure(1);
    plot(time1, angle1);
    hold on;
    plot(time2, angle2, '-y');
    hold off;
    %}
    
    t= 1;
    maxIndex1 = getFirstMaximum(time1, q1(1,:), 0, 0.01);
    maxIndex2 = getFirstMaximum(time2, q2(1,:), 0, 0.01);
    %disp(maxIndex1+1.6-maxIndex2);
    %time1 = time1 - maxIndex1-1.6;
    %time2 = time2 - maxIndex2;
    time2 = time2 -5.3220+0.7-0.57;
    euler1 = zeros(3, length(time2));
    euler2 = zeros(3, length(time2));
    euler0 = GetAnglesFromQuaternion(q0, 'optit');
    disp(euler0);
    %move = alignByLeastsquares(time1, q1, time2, q2, 15, 40);
    %time1 = time1 - move;
    q = funcSlerp(time1, time2, q1, q2);
    for i = 1:1:length(time2)
        %{
        if i == 1
            euler1(:,1) = [0;0;0];
        else
            for j = 1:1:3
                euler1(j,i) = euler1(j,i-1) + (time1(i)-time1(i-1))*rot1(j,i)*180/pi;
            end
        end
        %}
        euler1(:,i) = GetAnglesFromQuaternion(q(:,i),'optit');
    end
    
    for i = 1:1:length(time2)
        euler2(:,i) = GetAnglesFromQuaternion(q2(:,i),'optit');
    end
    deltq = zeros(4, length(time2));
    for i = 1:1:length(time2)
        deltq(:,i) = HamiltonProduct(inverse(q(:,i)), q2(:,i));
    end
    
    euler2(1,:) = -euler2(1,:);
    q2(1,:) = -q2(1,:);
    [newx1, splineq1] = interpolationBySpline(time1, q1, 10, 40, 0.001);
    [newx2, splineq2] = interpolationBySpline(time2, q2, 10, 40, 0.001);
    for i = 1:1:length(splineq1)
        splineEuler1(:,i) = GetAnglesFromQuaternion(splineq1(:,i),'optit');
    end
    for i = 1:1:length(splineq2)
        splineEuler2(:,i) = GetAnglesFromQuaternion(splineq2(:,i),'optit');
    end
    for i = 1:1:3
        figure(i);
        %plot(time2, euler1(i,:));
        hold on;
        %plot(time2, euler1(i,:));
        plot(newx1, splineEuler1(i,:),'*');
        hold off;
        hold on;
        %plot(time2, euler2(i,:));
        plot(newx2, splineEuler2(i,:),'*');
        hold off;
        
        xlim([10,40]);
    end
    
    %{
    [bAcc,aAcc] = butter(3,0.2,'low');
    AccF = zeros(3, length(time1));
    for i = 1:1:length(time1)
        if(i<=13)
            AccF(:,i)=MyFilter(bAcc,aAcc,acc1(:,:));
        else
            AccF(:,i)=MyFilter(bAcc,aAcc,acc1(:,i-13:i));
        end
    end
    plot(time2, acc2);
    figure(2);
    plot(time1, AccF);
   %}
end
function [result] = matrixMultiVector(a, b)
    result =  zeros(3,1);

    result(1) = a(1) * b(1) + a(2) * b(2) + a(3) * b(3);
    result(2) = a(4) * b(1) + a(5) * b(2) + a(6) * b(3);
    result(3) = a(7) * b(1) + a(8) * b(2) + a(9) * b(3);
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