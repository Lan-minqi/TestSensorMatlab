function [time, rotData, accData, magData, q] = readPhoneData( name ,settings)
  import java.text.SimpleDateFormat ;
  import java.util.Date ;
  import java.util.TimeZone ;
  import java.lang.Integer;
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
file=fopen(strcat('C:\Users\lmq\Desktop\sensor\phonedata\',name),'r');
if(file == -1)
    disp('File open error');
end
NUM = 100;
%NUM不能大
time = zeros(1, NUM);
rotData = zeros(3, NUM);
accData = zeros(3, NUM);
magData = zeros(3, NUM);
i = 0;
preTime = 0;
while 1
    tline=fgetl(file);
    if (tline == -1) ,break;end
    i = i+1;
    time(i) = str2double(tline);
    
    date = Date(time(i));
    formatter = SimpleDateFormat('HH:mm:ss:SSS');
    dateFormatted = formatter.format(date);
    timme = dateFormatted.split (':');
    hour = Integer.parseInt (timme(1).trim());
    min = Integer.parseInt (timme(2).trim());
    second = Integer.parseInt (timme(3).trim());
    millisecond = Integer.parseInt (timme(4).trim());
    time(i) = ((hour*60+min)*60)+second+millisecond/1000;
    if i > 1 && preTime >= time(i)
        i = i - 1;
        continue;
    end
    preTime = time(i);
    tline=deblank(fgetl(file));
    S = regexp(tline, ' ', 'split');
    for j = 1:1:3     
       rotData(j,i) = str2double(char(S(j)));
    end
    
    tline=deblank(fgetl(file));
    S = regexp(tline, ' ', 'split');
    for j = 1:1:3    
        accData(j,i) = str2double(char(S(j)));
    end
    magData(:,i) = zeros(3,1);
    
    tline=deblank(fgetl(file));
    S = regexp(tline, ' ', 'split');
    for j = 1:1:3    
        magData(j,i) = str2double(char(S(j)));
    end
    
end
fclose(file);

[accF, q] = getQuaternionByComplementary(time(1:i), rotData(:,1:i), accData(:,1:i), magData(:,1:i), settings);

end


