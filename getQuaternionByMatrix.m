function [qw, qx, qy, qz] = getQuaternionByMatrix(m)
    tr = m(1) + m(5) + m(9);
  %{
    00 1
    01 2
    02 3
    10 4
    11 5
    12 6
    20 7
    21 8
    22 9 
    %}
if(tr > 0)
  S = sqrt(tr + 1.0)* 2;
  qw = 0.25 * S;
  qx =(m(8)-m(6)) / S;
  qy =(m(3)-m(7)) / S; 
  qz =(m(4)-m(2)) / S; 
elseif ((m(1)> m(5)) && (m(1)> m(9)))
  S = sqrt(1.0 + m(1)  -  m(5)  -  m(9))* 2;
  qw = (m(8) - m(6)) / S;
  qx = 0.25 * S;
  qy =(m(2) + m(4)) / S; 
  qz =(m(3) + m(7)) / S; 
elseif (m(5) > m(9))
  S = sqrt(1.0 + m(5) - m(1) -  m(9))* 2; 
  qw = (m(3) - m(7)) / S;
  qx = (m(2) + m(4)) / S; 
  qy = 0.25 * S;
  qz = (m(6) + m(8)) / S; 
else 
  S = sqrt(1.0 + m(9)  -  m(1)  -  m(5))* 2; 
  qw = (m(4) - m(2)) / S;
  qx = (m(3) + m(7)) / S;
  qy = (m(6) + m(8)) / S;
  qz = 0.25 * S;
end