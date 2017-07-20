function [] = receiveFromPhoneScalable()
%实时画图 传过来的是原始的传感器数据 可扩展：可以自由添加滤波器
%与TestSensor3配合 实时观察
    interfaceObject = tcpip('0.0.0.0', 12345, 'NetworkRole', 'server');
    set(interfaceObject,'InputBufferSize',3000);
    %'CloseRequestFcn',{@localCloseFigure,interfaceObject}表示关闭图像要调用的函数
    figureHandle = figure('NumberTitle','off','Name','Live Data Stream Plot',...
    'Color',[0 0 0],'CloseRequestFcn',{@localCloseFigure,interfaceObject});
    axesHandle = axes('Parent',figureHandle,'YGrid','on',...
            'YColor',[0.9725 0.9725 0.9725],'XGrid','on',...
            'XColor',[0.9725 0.9725 0.9725],'Color',[0 0 0]);
    xlabel(axesHandle,'Time');
    ylabel(axesHandle,'Value');
    %ylim([-10,10]);
    
     %Initialize the plot and hold the settings on
     hold on;
     xHandle = plot(axesHandle,0,'-y','LineWidth',1);
     hold on;
     %绕x轴 pitch 紫
     yHandle = plot(axesHandle,0,'-y','LineWidth',1);
     set(yHandle, 'Color',[1 0 1]);
     hold on;
     %绕y轴 roll 绿
     zHandle = plot(axesHandle,0,'-y','LineWidth',1);
     set(zHandle, 'Color',[0 1 1]);
     %绕z轴 yaw 黄
     fopen(interfaceObject);
     hasInitialOrientation = 0;
     stateInitialized = 0;
     oldtime = 0;
     global initialRotationMatrix;
     global currentRotationMatrix;
     global gyroOrientation;
     global accMagOrientation;
     global fusedOrientation;
     global FILTER_COEFFICIENT;
     global time;
     initialRotationMatrix = zeros(9,1);
     currentRotationMatrix = zeros(9,1);
     gyroOrientation = zeros(3,1);
     accMagOrientation = zeros(3,1);
     fusedOrientation = zeros(3,1);
     FILTER_COEFFICIENT = 0.98;
     time = zeros(1,1);
     while 1
        if ~ isvalid(interfaceObject)
            break;
        end
        [hasInitialOrientation, stateInitialized, oldtime] = localReadAndPlot(interfaceObject, [xHandle,yHandle,zHandle], 100, hasInitialOrientation, stateInitialized, oldtime);
        snapnow;
     end
     %save('test.mat', 'Eulers');
end
 function [hasInitialOrientation, stateInitialized, oldtime] = localReadAndPlot(interfaceObject,handles, xPointNum,hasInitialOrientation,stateInitialized,oldtime,~)
       % Read the desired number of data bytes
       rot = zeros(3,1);
       accel = zeros(3, 1);
       magnet = zeros(3,1);
       i = 1;
       global initialRotationMatrix;
       global currentRotationMatrix;
       global gyroOrientation;
       global accMagOrientation;
       global fusedOrientation;
       global FILTER_COEFFICIENT;
       global time;
       while (get(interfaceObject, 'BytesAvailable') > 0) 
           a = fscanf(interfaceObject);
           %disp(a);
           S = regexp(a, ' ', 'split');
           time = [time str2double(S(1))];
           for j = 1:1:3
               rot(j,i) = str2double(S(1+j));
           end
           for j = 1:1:3
               accel(j,i) = str2double(S(4+j));
           end
           for j = 1:1:3
               magnet(j,i) = str2double(S(7+j));
           end
           
           if (~hasInitialOrientation)
                [hasInitialOrientation,initialRotationMatrix] = getRotationMatrix(initialRotationMatrix, accel, magnet);
                if(~hasInitialOrientation) 
                    return;end
           end
            if (~stateInitialized)
                currentRotationMatrix = initialRotationMatrix;
                
                stateInitialized = true;
            end

            if (stateInitialized)
                len = length(time);
                if(len == 1)
                    return;
                end 
                dT = (time(end) - time(end-1)) * 0.001;
                if(dT <= 0) 
                    return
                end
                
                axisX = rot(1);
                axisY = rot(2);
                axisZ = rot(3);

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
                %disp(currentRotationMatrix);
                gyroOrientation(:,len) = getOrientation(currentRotationMatrix);

                oneMinusCoeff = 1.0 - FILTER_COEFFICIENT;
                %cal accMagOrientation
                accMagOrientation(:,len) = [0,0,0];
                rotationMatrix = zeros(9,1);
                [ok, rotationMatrix] = getRotationMatrix(rotationMatrix, accel, magnet);
                 if(ok) 
                    accMagOrientation(:,len) = getOrientation(rotationMatrix);
                 end
                fusedOrientation(1, len) = FILTER_COEFFICIENT * gyroOrientation(1, len) + oneMinusCoeff * accMagOrientation(1, len);

                fusedOrientation(2, len) = FILTER_COEFFICIENT * gyroOrientation(2, len) + oneMinusCoeff * accMagOrientation(2, len);

                fusedOrientation(3, len) = FILTER_COEFFICIENT * gyroOrientation(3, len) + oneMinusCoeff * accMagOrientation(3, len);

                currentRotationMatrix = getRotationMatrixFromOrientation(fusedOrientation(:,len));
                gyroOrientation(:,len) = fusedOrientation(:,len);            
                gyroOrientation(1, len)=gyroOrientation(1, len)*180/pi;
                gyroOrientation(2, len)=gyroOrientation(2, len)*180/pi;
                gyroOrientation(3, len)=gyroOrientation(3, len)*180/pi;
                %disp(gyroOrientation(:, len));
            end
           
           i = i+1;
       end
       
       refreshImage(time, gyroOrientation, xPointNum, handles);
 end 
 %Implement the close figure callback
 function localCloseFigure(figureHandle,~, interfaceObject)
     disp('stop');
     
     fclose(interfaceObject);
     delete(interfaceObject);
     clear interfaceObject;
     delete(figureHandle);
 end
     
function  refreshImage(time, rot, xPointNum, handles)
    X = get(handles(1), 'XData');
    Y = get(handles(1), 'YData');
    Y2 = get(handles(2), 'YData');
    Y3 = get(handles(3), 'YData');
    
   if isempty(X) || length(rot(1,:)) <= xPointNum
       %disp(length(time));
       %disp(length(rot(1,:)));
       X = time;
       Y = rot(1,:);
       Y2 = rot(2,:);
       Y3 = rot(3,:);
   else
       %disp(length(time));
       %disp(length(rot(1,:)));
       X = time(end-xPointNum+1:end);
       Y = rot(1,end-xPointNum+1:end);
       Y2 = rot(2,end-xPointNum+1:end);
       Y3 = rot(3,end-xPointNum+1:end);
   end

   % Update the plot

   set(handles(1),'XData',X,'YData',Y); 
   set(handles(2),'XData',X,'YData',Y2); 
   set(handles(3),'XData',X,'YData',Y3); 
end
