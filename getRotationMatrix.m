function [boolval, R] = getRotationMatrix(R, gravity, geomagnetic)
        Ax = gravity(1);
        Ay = gravity(2);
        Az = gravity(3);

        normsqA = (Ax*Ax + Ay*Ay + Az*Az);
        g = 9.81;
        freeFallGravitySquared = 0.01 * g * g;
        if (normsqA < freeFallGravitySquared) 
            %gravity less than 10% of normal value
            boolval = 0;
            return;
        end

        Ex = geomagnetic(1);
        Ey = geomagnetic(2);
        Ez = geomagnetic(3);
        Hx = Ey*Az - Ez*Ay;
        Hy = Ez*Ax - Ex*Az;
        Hz = Ex*Ay - Ey*Ax;
        normH = sqrt(Hx*Hx + Hy*Hy + Hz*Hz);

        if (normH < 0.1) 
            boolval = 0;
            return;
        end
        invH = 1.0 / normH;
        Hx = Hx*invH;
        Hy = Hy*invH;
        Hz = Hz*invH;
        invA = 1.0 / sqrt(Ax*Ax + Ay*Ay + Az*Az);
        Ax = Ax*invA;
        Ay = Ay*invA;
        Az = Az*invA;
        Mx = Ay*Hz - Az*Hy;
        My = Az*Hx - Ax*Hz;
        Mz = Ax*Hy - Ay*Hx;
        if (length(R) == 9)
            R(1) = Hx;     R(2) = Hy;     R(3) = Hz;
            R(4) = Mx;     R(5) = My;     R(6) = Mz;
            R(7) = Ax;     R(8) = Ay;     R(9) = Az;
        end
        
        boolval = 1;
        return;
    