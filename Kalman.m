function [] = Kalman()
    interfaceObject = tcpip('0.0.0.0', 12345, 'NetworkRole', 'server');
    set(interfaceObject,'InputBufferSize',3000);
    acqSize=1500;

    %Gyroscope statistics
    Offset=[0.052813,0.032632,0.02781]'; 
    %Offset=[-3.6982,-3.3570,-2.5909]';
    gravity = 9.80665;
    var=[(0.5647/180*pi)^2 (0.5674/180*pi)^2 (0.5394/180*pi)^2]';
    %var=[(0.7698/180*pi)^2 (0.4925/180*pi)^2 (0.5144/180*pi)^2]';

    %Acquisition variables
    GyroRate=zeros(3,acqSize);
    Acc=zeros(3,acqSize);
    g = zeros(3, acqSize);
    g(3,1) = 1;
    accWithout = zeros(3, acqSize);
    Magn=zeros(3,acqSize);
    Angles=zeros(3,acqSize);
    AccF=zeros(3,acqSize);
    MagnF=zeros(3,acqSize);
    mu=zeros(1,acqSize);
    dqnorm=zeros(1,acqSize);
    dq=zeros(4,acqSize);

    qUpdate=zeros(4,acqSize);
    %Initial quaternion values
    qUpdate(:,1)=[1 0 0 0]';
    
    %Observation vector
    qOsserv=zeros(4,acqSize);
    qOsserv(:,1)=[1 0 0 0]';

    %----KALMAN MATRIXES
    Q1=[var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1)];
    Q2=[-var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1)];
    Q3=[-var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1)];
    Q4=[var(1,1)-var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1)];
    Qmatrix=[Q1;Q2;Q3;Q4];

    H=eye(4,4);

    sigmaR=[0.01 0.01 0.01 0.01]';
    R=[sigmaR(1,1) 0 0 0;0 sigmaR(2,1) 0 0;0 0 sigmaR(3,1) 0;0 0 0 sigmaR(4,1)];

    qPredicted=zeros(4,acqSize);
    %qPredicted(:,1)=[0.5 0.5 0.5 0.5]';
    qPredicted(:,1)=[1 0 0 0]';
    P_Update=eye(4,4)*2;
    %----------
    t=[0];

    i=1;
    dt=0;

    [bAcc,aAcc] = butter(3,0.0075,'low');
    [bMagn,aMagn] = butter(2,0.06,'low');

    magnF_Length=13;
    accF_Length=13;

    %Bring up the filters
    disp('Connecting');
    fopen(interfaceObject);
    if(interfaceObject.status=='open')
        disp('Connected');
    end
    while 1
        if ~ isvalid(interfaceObject) 
            break;
        end
       
        if (i == acqSize+1)
            break;
        end
        % Read the desired number of data bytes
       time = zeros(1, 1);
       rot = zeros(3,1);
       acc = zeros(3, 1);
       magn = zeros(3,1);
       while (get(interfaceObject, 'BytesAvailable') > 0) 
           disp(i);
           a = fscanf(interfaceObject);
           S = regexp(a, ' ', 'split');
           time = str2double(S(2));
           for j = 1:1:3
               rot(j) = str2double(S(2+j));
           end
           for j = 1:1:3
               acc(j) = str2double(S(5+j));
           end
           for j = 1:1:3
               magn(j) = str2double(S(8+j));
           end
           
           if(i<=accF_Length+4)
                if(i>1)
                    t=[t time];
                    dt = (t(i) - t(i-1))/1000; 
                else
                    t(1) = time;
                    dt = 0.02;
                end
                t0 = tic;

                Acc(1,i)=acc(1);
                Acc(2,i)=acc(2);
                Acc(3,i)=acc(3);
                accWithout(:,i) = Acc(:,i) - g(:,i)*gravity;
                Magn(1,i)=magn(1);
                Magn(2,i)=magn(2);
                Magn(3,i)=magn(3);
                GyroRate(1,i)=(rot(1)-Offset(1,1)*dt);
                GyroRate(2,i)=(rot(2)-Offset(2,1)*dt);
                GyroRate(3,i)=(rot(3)-Offset(3,1)*dt);

                Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
                Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
                if(i<=accF_Length)
                    AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,:));
                else
                    AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
                end
                if(i<=magnF_Length)
                    MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,:));
                else
                    MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
                end
                MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
                AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
                i=i+1;
                qPredicted(:,i)=[0.5 0.5 0.5 0.5]';
                qUpdate(:,i)=qPredicted(:,i);
                
                g(1,i+1) = 2 *(qUpdate(2,i) * qUpdate(4,i)  -  qUpdate(1,i) * qUpdate (3,i));
                g(2,i+1) = 2 *(qUpdate(1,i) * qUpdate(2,i)  +  qUpdate(3,i) * qUpdate (4,i));
                g(3,i+1) = qUpdate(1,i) * qUpdate(1,i)  -  qUpdate(2,i) * qUpdate (2,i) - qUpdate(3,i)*qUpdate(3,i) + qUpdate(4,i)*qUpdate(4,i);
                
                qOsserv(:,i)=qPredicted(:,i);
           elseif (i<=acqSize)
                if(i>2)
                    t=[t time];
                    dt = (t(i) - t(i-1))/1000;
                else
                    t(1) = time;
                    dt = 0.02;
                end
                %dt=0.015;        
                    t0 = tic;

                Acc(1,i)=acc(1);
                Acc(2,i)=acc(2);
                Acc(3,i)=acc(3);
                accWithout(:,i) = Acc(:,i) - g(:,i)*gravity;
                Magn(1,i)=magn(1);
                Magn(2,i)=magn(2);
                Magn(3,i)=magn(3);
                GyroRate(1,i)=(rot(1)-Offset(1,1)*dt);
                GyroRate(2,i)=(-rot(2)+Offset(2,1)*dt);
                GyroRate(3,i)=(rot(3)-Offset(3,1)*dt);
                

                GyroRate(1,i)=(GyroRate(1,i)+GyroRate(1,i-1))/2;
                GyroRate(2,i)=(GyroRate(2,i)+GyroRate(2,i-1))/2;
                GyroRate(3,i)=(GyroRate(3,i)+GyroRate(3,i-1))/2;

                %Normalization and filtering
                Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
                Magn(:,i)=Magn(:,i)/norm(Magn(:,i));

                AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
                MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));

                MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
                AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
                %----End Acquisition

                %OBSERVATION COMPUTING

                %Gradient Descent
                dq(:,i)=0.5*(QuaternionProduct(qUpdate(:,i-1),[0 GyroRate(1,i) GyroRate(2,i) GyroRate(3,i)]'));
                dqnorm(1,i)=norm(dq(:,i));
                mu(1,i)=10*dqnorm(1,i)*dt;
                qOsserv(:,i)=GradientDescent(AccF(:,i),MagnF(:,i),qOsserv(:,i-1),mu(1,i));
                qOsserv(:,i)=qOsserv(:,i)/norm(qOsserv(:,i));
                %END OSSERVATION COMPUTING

                %KALMAN FILTERING
                const=dt/2;
                %F matrix computing
                F1=[1 -const*GyroRate(1,i) -const*GyroRate(2,i) -const*GyroRate(3,i)];
                F2=[const*GyroRate(1,i) 1 const*GyroRate(3,i) -const*GyroRate(2,i)];
                F3=[const*GyroRate(2,i) -const*GyroRate(3,i) 1 const*GyroRate(1,i)];
                F4=[-const*GyroRate(3,i) const*GyroRate(2,i) -const*GyroRate(1,i) 1];

                F=[F1;F2;F3;F4];
                qPredicted(:,i)=F*qUpdate(:,i-1);

                Q=Qmatrix;

                P_Predicted=F*P_Update*F'+Q;

                K=P_Predicted*H'*(H*P_Predicted*H'+R)^-1;
                qUpdate(:,i)=qPredicted(:,i)+K*(qOsserv(:,i)-H*qPredicted(:,i));
                qUpdate(:,i)=qUpdate(:,i)/norm(qUpdate(:,i));
                
                g(1,i+1) = 2 *(qUpdate(2,i) * qUpdate(4,i)  -  qUpdate(1,i) * qUpdate (3,i));
                g(2,i+1) = 2 *(qUpdate(1,i) * qUpdate(2,i)  +  qUpdate(3,i) * qUpdate (4,i));
                g(3,i+1) = qUpdate(1,i) * qUpdate(1,i)  -  qUpdate(2,i) * qUpdate (2,i) - qUpdate(3,i)*qUpdate(3,i) + qUpdate(4,i)*qUpdate(4,i);
                
                P_Update=(eye(4,4)-K*H)*P_Predicted;
                Angles(:,i)=GetAnglesFromQuaternion(qUpdate(:,i));

                %END KALMAN FILTERING
                i=i+1;
            end
       end
    end
     fclose(interfaceObject);
     delete(interfaceObject);
     clear interfaceObject;
     figure;
    subplot(3,1,1);plot(t,Acc(1,:),'b',t,AccF(1,:),'r',t,Magn(1,:),'g',t,MagnF(1,:),'c');legend('AccX','AccFX','MagnX','MagnFX');grid;
    subplot(3,1,2);plot(t,Acc(2,:),'b',t,AccF(2,:),'r',t,Magn(2,:),'g',t,MagnF(2,:),'c');legend('AcY','AccFY','MagnY','MagnFY');grid;
    subplot(3,1,3);plot(t,Acc(3,:),'b',t,AccF(3,:),'r',t,Magn(3,:),'g',t,MagnF(3,:),'c');legend('AccZ','AccFZ','MagnZ','MagnFZ');grid;
%{
    figure;
    subplot(4,1,1);plot(t,qOsserv(1,1:acqSize));grid;legend('q0 Observed');
    subplot(4,1,2);plot(t,qOsserv(2,1:acqSize));grid;legend('q1 Observed');
    subplot(4,1,3);plot(t,qOsserv(3,1:acqSize));grid;legend('q2 Observed');
    subplot(4,1,4);plot(t,qOsserv(4,1:acqSize));grid;legend('q3 Observed');
%}
    figure;
    subplot(4,1,1);plot(t,qUpdate(1,1:acqSize));hold on;plot(t,qOsserv(1,1:acqSize),'r');grid;legend('q0 Estimated','q0 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,2);plot(t,qUpdate(2,1:acqSize));hold on;plot(t,qOsserv(2,1:acqSize),'r');grid;legend('q1 Estimated','q1 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,3);plot(t,qUpdate(3,1:acqSize));hold on;plot(t,qOsserv(3,1:acqSize),'r');grid;legend('q2 Estimated','q2 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,4);plot(t,qUpdate(4,1:acqSize));hold on;plot(t,qOsserv(4,1:acqSize),'r');grid;legend('q3 Estimated','q3 Observed');xlabel('time (sec)');ylabel('Quaternion value');    
    
    figure;
    subplot(3,1,1);plot(t,Angles(1,1:acqSize));grid;legend('Roll');xlabel('time (sec)');ylabel('Angle (deg)');
    subplot(3,1,2);plot(t,Angles(2,1:acqSize));grid;legend('Pitch');xlabel('time (sec)');ylabel('Angle (deg)');
    subplot(3,1,3);plot(t,Angles(3,1:acqSize));grid;legend('Yaw');xlabel('time (sec)');ylabel('Angle (deg)');
    
    %{
    figure;
    subplot(3,1,1);plot(t,accWithout(1,1:acqSize));grid;legend('Roll');xlabel('time (sec)');ylabel('Angle (deg)');
    subplot(3,1,2);plot(t,accWithout(2,1:acqSize));grid;legend('Pitch');xlabel('time (sec)');ylabel('Angle (deg)');
    subplot(3,1,3);plot(t,accWithout(3,1:acqSize));grid;legend('Yaw');xlabel('time (sec)');ylabel('Angle (deg)');
    %}
end







