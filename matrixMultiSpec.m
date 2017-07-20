function [result] = matrixMultiSpec(a, b)
    %返回a乘以b的转置
    result = zeros(9,1);

    result(1) = a(1) * b(1) + a(2) * b(2) + a(3) * b(3);
    result(2) = a(1) * b(4) + a(2) * b(5) + a(3) * b(6);
    result(3) = a(1) * b(7) + a(2) * b(8) + a(3) * b(9);

    result(4) = a(4) * b(1) + a(5) * b(2) + a(6) * b(3);
    result(5) = a(4) * b(4) + a(5) * b(5) + a(6) * b(6);
    result(6) = a(4) * b(7) + a(5) * b(8) + a(6) * b(9);

    result(7) = a(7) * b(1) + a(8) * b(2) + a(9) * b(3);
    result(8) = a(7) * b(4) + a(8) * b(5) + a(9) * b(6);
    result(9) = a(7) * b(7) + a(8) * b(8) + a(9) * b(9);
