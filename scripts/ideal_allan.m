clc
close all
clear

size = 1000000;
a_tmp = linspace(0,1000,size)';
a = a_tmp(2:size,:);
b = a.^(-2)+a.^(-1)+1+a.^(1)+a.^(2);

figure
loglog(a, sqrt(b) , '-');
hold on;  
grid on;
mea([1:size-1],1)= mean(b);
loglog(a, sqrt(mea) , '-');

