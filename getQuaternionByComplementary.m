function [AccF, q] = getQuaternionByComplementary(time, rot, accel, magnet, settings)
% TestSensor2保存的原始数据得到四元数
% initialRotationMatrix需要一段时间来确定，保证正确
%欧拉角旋转顺序 -z x y
     useButter = 0;
     hasInitialOrientation = 0;
     stateInitialized = 0;
     len = length(time);
     initialRotationMatrix = zeros(9,1);
     currentRotationMatrix = zeros(9,1);
     gyroOrientation = zeros(3,len);
     q = zeros(4, len);
     accMagOrientation = zeros(3,len);
     fusedOrientation = zeros(3,len);
     if useButter
        [bAcc,aAcc] = butter(3,0.2,'low');
        [bMagn,aMagn] = butter(2,0.3,'low');
     end
     AccF=zeros(3,len);
     MagnF=zeros(3,len);
     magnF_Length=13;
     accF_Length=13;
     FILTER_COEFFICIENT = 0.98;
     for i = 1:1:len
       if (~hasInitialOrientation)
            [hasInitialOrientation,initialRotationMatrix] = getRotationMatrix(initialRotationMatrix, accel, magnet);
            if(~hasInitialOrientation) 
                continue;end
       end
        if (~stateInitialized)
            currentRotationMatrix = initialRotationMatrix;
            stateInitialized = true;
        end

        if (stateInitialized)
            if(i == 1)
                continue;
            end 
            dT = time(i) - time(i-1);
            if(dT <= 0) 
                disp('error');
                continue;
            end

            axisX = rot(1,i);
            axisY = rot(2,i);
            axisZ = rot(3,i);

            omegaMagnitude = sqrt(axisX * axisX + axisY* axisY + axisZ * axisZ);
            axisX = axisX/omegaMagnitude;
            axisY = axisY/omegaMagnitude;
            axisZ = axisZ/omegaMagnitude;

            thetaOverTwo = omegaMagnitude * dT / 2.0;

            sinThetaOverTwo = sin(thetaOverTwo);
            cosThetaOverTwo = cos(thetaOverTwo);

            deltaRotationVector = zeros(4,1);
            deltaRotationVector(1) = sinThetaOverTwo * axisX;
            deltaRotationVector(2) = sinThetaOverTwo * axisY;
            deltaRotationVector(3) = sinThetaOverTwo * axisZ;
            deltaRotationVector(4) = cosThetaOverTwo;

            deltaRotationMatrix = zeros(9,1);
           
            deltaRotationMatrix = getRotationMatrixFromVector(deltaRotationMatrix,deltaRotationVector);

            currentRotationMatrix = matrixMultiplication(currentRotationMatrix,deltaRotationMatrix);
            
            gyroOrientation(:,i) = getOrientation(currentRotationMatrix);

            oneMinusCoeff = 1.0 - FILTER_COEFFICIENT;
            %cal accMagOrientation
            accMagOrientation(:,i) = [0,0,0];
            rotationMatrix = zeros(9,1);
            if useButter
                if(i<=accF_Length)
                    AccF(:,i)=MyFilter(bAcc,aAcc,accel(:,:));
                else
                    AccF(:,i)=MyFilter(bAcc,aAcc,accel(:,i-accF_Length:i));
                end
                if(i<=magnF_Length)
                    MagnF(:,i)=MyFilter(bMagn,aMagn,magnet(:,:));
                else
                    MagnF(:,i)=MyFilter(bMagn,aMagn,magnet(:,i-magnF_Length:i));
                end
            end
            [ok, rotationMatrix] = getRotationMatrix(rotationMatrix, accel, magnet);
             if(ok) 
                accMagOrientation(:,i) = getOrientation(rotationMatrix);
             end
            fusedOrientation(:, i) = FILTER_COEFFICIENT * gyroOrientation(:, i) + oneMinusCoeff * accMagOrientation(:, i);
            currentRotationMatrix = getRotationMatrixFromOrientation(fusedOrientation(:,i));            
            gyroOrientation(:,i) = fusedOrientation(:, i);
            gyroOrientation(:,i)=gyroOrientation(:, i)*180/pi;
            %{
            %由欧拉角得到旋转四元数
            t0 = sin(gyroOrientation(1,i)/2);
            t1 = cos(gyroOrientation(1,i)/2);
            t2 = sin(gyroOrientation(2,i)/2);
            t3 = cos(gyroOrientation(2,i)/2);
            t4 = sin(gyroOrientation(3,i)/2);
            t5 = cos(gyroOrientation(3,i)/2);
            q(1,i) = -t0*t4*t3 + t1*t2*t5;
            q(2,i) = t0*t2*t5 + t1*t3*t4;
            q(3,i) = t0*t3*t5 + t1*t2*t4;
            q(4,i) = -t0*t2*t4 + t1*t3*t5;
            %}
            if settings == 'relative'
                [q(1,i),q(2,i),q(3,i),q(4,i)] = getQuaternionByMatrix(matrixMultiSpec(currentRotationMatrix,initialRotationMatrix));
            elseif settings == 'absolute'
                [q(1,i),q(2,i),q(3,i),q(4,i)] = getQuaternionByMatrix(currentRotationMatrix);
            end
       end
     end
     %{
     for i = 1:1:3
        figure(i+3);
        plot(time, gyroOrientation(i,:));
       %xlim([5,35]);
    end
     %}
end

