function [values] = getOrientation(R) 
    values = zeros(3,1);
    values(1) = atan2(R(2), R(5));
    values(2) = asin(-R(8));
    values(3) = atan2(-R(7), R(9));