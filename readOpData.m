function [time, accData, q] = readOpData( name )

%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
file=fopen(strcat('C:\Users\lmq\Desktop\sensor\opdata\',name),'r');
if(file == -1)
    disp('File open error');
end
NUM = 100;
time = zeros(1, NUM);
q = zeros(4, NUM);
accData = zeros(3, NUM);
i = 0;
startTime = 0;
preTime = 0;
while 1
    tline=fgetl(file);
    if (tline == -1) ,break;end
    tline=deblank(tline);
    S = regexp(tline, '\s', 'split');
    i = i+1;
    time(i) = str2double(S(1))/1000;
    
    if (i == 1) 
        startTime = time(i);
    end
    %if(preTime >0 && str2double(char(S(1)))/1000 == preTime),i = i-1;end
    
    if time(i) == 0
        i = i - 1;
        break;
    end
    %disp(time);
    preTime = time(i);
    for j = 1:1:3     
        accData(j,i) = str2double(char(S(1+j)));
    end
    
    for j = 1:1:4    
        q(j,i) = str2double(char(S(4+j)));
    end
end
fclose(file);

%{
figure(2);
plot(time(1:i), q(1,1:i));
hold on;
plot(time(1:i), q(2,1:i),'-r');
hold off;
hold on;
plot(time(1:i), q(3,1:i),'-g');
hold off;
hold on;
plot(time(1:i), q(4,1:i),'-y');
hold off;
%}
end

