function [R] =  getRotationMatrixFromVector(R, rotationVector) 
    q0 = 0;
    q1 = rotationVector(1);
    q2 = rotationVector(2);
    q3 = rotationVector(3);

    if (length(rotationVector) >= 4)
        q0 = rotationVector(4);
    else
        q0 = 1 - q1*q1 - q2*q2 - q3*q3;
        if(q0 > 0)
            q0 = sqrt(q0);
        else
            q0 = 0;
        end
    end

    sq_q1 = 2 * q1 * q1;
    sq_q2 = 2 * q2 * q2;
    sq_q3 = 2 * q3 * q3;
    q1_q2 = 2 * q1 * q2;
    q3_q0 = 2 * q3 * q0;
    q1_q3 = 2 * q1 * q3;
    q2_q0 = 2 * q2 * q0;
    q2_q3 = 2 * q2 * q3;
    q1_q0 = 2 * q1 * q0;

    R(1) = 1 - sq_q2 - sq_q3;
    R(2) = q1_q2 - q3_q0;
    R(3) = q1_q3 + q2_q0;

    R(4) = q1_q2 + q3_q0;
    R(5) = 1 - sq_q1 - sq_q3;
    R(6) = q2_q3 - q1_q0;

    R(7) = q1_q3 - q2_q0;
    R(8) = q2_q3 + q1_q0;
    R(9) = 1 - sq_q1 - sq_q2;