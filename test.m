function [] = test()
    q0 = [sqrt(0.5), 0, 0, sqrt(0.5)];
    %q0 = [1,0,0,0];
    q1 = [0, 0, 0, 1];
    q2 = [-sqrt(0.5), 0, 0, sqrt(0.5)];
    q3 = Hamilton0(Hamilton0(q0, [0, q2(2:4)]), inverse(q0));
    q2(2:4) = q3(2:4);
    q2= Hamilton0(q2,q0);
    disp(q2);
end
function [result] = matrixMultiVector(a, b)
    result =  zeros(3,1);

    result(1) = a(1) * b(1) + a(2) * b(2) + a(3) * b(3);
    result(2) = a(4) * b(1) + a(5) * b(2) + a(6) * b(3);
    result(3) = a(7) * b(1) + a(8) * b(2) + a(9) * b(3);
end
function [result] = inverse(a)
    %a为四元数 qw qx qy qz
    result = zeros(4);
    x = a(1)^2 + a(2)^2 + a(3)^2 + a(4)^2;
    result(1) = a(1)/x;
    result(2) = -a(2)/x;
    result(3) = -a(3)/x;
    result(4) = -a(4)/x;
end
function [q] = Hamilton0(a, b)
%四元数乘法
%a为4*N四元数向量, b为4*1四元数向量
%w x y z
    q = zeros(4,1);
    q(1) = a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
    q(2) = a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
    q(3) = a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
    q(4) = a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1);
end