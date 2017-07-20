function [ time] = convertTime( tline )
S = regexp(tline, ':', 'split');
hour = str2double(char(S(1)));
minute = str2double(char(S(2)));
second = str2double(char(S(3)));
mico = str2double(char(S(4)));
time =  ((hour*60+minute)*60+second)+mico/1000;
