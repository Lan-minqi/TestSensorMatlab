function [result] = matrixMultiplication(a, b)
        result =  zeros(9,1);

        result(1) = a(1) * b(1) + a(2) * b(4) + a(3) * b(7);
        result(2) = a(1) * b(2) + a(2) * b(5) + a(3) * b(8);
        result(3) = a(1) * b(3) + a(2) * b(6) + a(3) * b(9);

        result(4) = a(4) * b(1) + a(5) * b(4) + a(6) * b(7);
        result(5) = a(4) * b(2) + a(5) * b(5) + a(6) * b(8);
        result(6) = a(4) * b(3) + a(5) * b(6) + a(6) * b(9);

        result(7) = a(7) * b(1) + a(8) * b(4) + a(9) * b(7);
        result(8) = a(7) * b(2) + a(8) * b(5) + a(9) * b(8);
        result(9) = a(7) * b(3) + a(8) * b(6) + a(9) * b(9);
