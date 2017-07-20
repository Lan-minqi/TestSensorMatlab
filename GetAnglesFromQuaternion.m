function [Angles] = GetAnglesFromQuaternion(q, source)
    q0=q(1);
    q1=q(2);
    q2=q(3);
    q3=q(4);
    %{
    from android 
    values[0] = (float)Math.atan2(R[1], R[4]);
            values[1] = (float)Math.asin(-R[7]);
            values[2] = (float)Math.atan2(-R[6], R[8]);
    %}
    %{
    Angles(1,1)=atan2(2*(q2*q3)-2*q0*q1,2*q0^2+2*q3^2-1)*180/pi;
    Angles(2,1)=-asind(2*q1*q3+2*q0*q2);
    Angles(3,1)=atan2(2*q1*q2-2*q0*q3,2*q0^2+2*q1^2-1)*180/pi;
    %}
    Angles(1)=atan2(2*(q1*q2)-2*q0*q3,1-2*q1^2-2*q3^2)*180/pi;
    Angles(2)=-asind(2*q2*q3+2*q0*q1);
    Angles(3)=atan2(2*q0*q2-2*q1*q3,1-2*q2^2-2*q1^2)*180/pi;
    if source == 'phone'
        %q0 q1 q2 q3对应qw qx qy qz
        %-z x y
        
    elseif source == 'optit'
        %q0 q1 q2 q3对应(手机坐标系)qw qx qy qz
        %z x y
        Angles(1) = -Angles(1);
    end
end

