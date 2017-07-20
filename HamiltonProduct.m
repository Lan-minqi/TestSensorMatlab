function [q] = HamiltonProduct(a, b)
%四元数乘法
%a为4*N四元数向量, b为4*1四元数向量
%w x y z
[x, y] =size(a);
if(y > 1)
    q = zeros(4, length(a));
    for i = 1:1:length(a)
        q(1,i) = a(1,i)*b(1) - a(2,i)*b(2) - a(3,i)*b(3) - a(4,i)*b(4);
        q(2,i) = a(1,i)*b(2) + a(2,i)*b(1) + a(3,i)*b(4) - a(4,i)*b(3);
        q(3,i) = a(1,i)*b(3) - a(2,i)*b(4) + a(3,i)*b(1) + a(4,i)*b(2);
        q(4,i) = a(1,i)*b(4) + a(2,i)*b(3) - a(3,i)*b(2) + a(4,i)*b(1);
    end
else
    q = zeros(4, 1);
    q(1) = a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
    q(2) = a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
    q(3) = a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
    q(4) = a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1);
end
end