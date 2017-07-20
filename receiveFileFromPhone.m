function [] = receiveFromPhone()
%接收TestSensor2的文件并且显示
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
     disp('connected');
     reachEnd = 0;
     while 1
        if ~ isvalid(interfaceObject)
            break;
        end
        if reachEnd == 1
            break;
        end
       while (get(interfaceObject, 'BytesAvailable') > 0) 
           a = fscanf(interfaceObject);
           disp(a);
           S = regexp(a, ' ', 'split');
           if(strcmp(a, 'END')) 
               disp(111);
               reachEnd = 1;
               break;
           end
           
       end
     end
   
 function [] = localReadAndPlot(interfaceObject,handles, xPointNum,~)
       % Read the desired number of data bytes
       
     
           
       % Update the plot
       %{
       set(handles(1),'XData',X,'YData',Y); 
       set(handles(2),'XData',X,'YData',Y2); 
       set(handles(3),'XData',X,'YData',Y3); 
       %}
 %Implement the close figure callback
 function localCloseFigure(figureHandle,~, interfaceObject)
     disp('stop');
     fclose(interfaceObject);
     delete(interfaceObject);
     clear interfaceObject;
     delete(figureHandle);
 
    
