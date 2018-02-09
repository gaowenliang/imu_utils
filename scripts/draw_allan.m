clear 
close all

dt = dlmread('data/data_A3_acc_t.txt');         %从文本中读取数据，单位：deg/s，速率：100Hz
data_x = dlmread('data/data_A3_acc_x.txt'); 
data_y= dlmread('data/data_A3_acc_y.txt'); 
data_z = dlmread('data/data_A3_acc_z.txt'); 

data_draw=[data_x data_y data_z] ;
figure
loglog(dt, data_draw , '*');
xlabel('time:sec');                 %添加x轴标签
% ylabel('Sigma:deg/h');              %添加y轴标签
legend('acc x','acc y','acc z');       %添加标注
grid on;                            %添加网格线
hold on;                            %使图像不被覆盖

CC(1, :) = fit_allan(sqrt(dt'), data_x', 2)';   %拟合
CC(2, :) = fit_allan(sqrt(dt'), data_y', 2)';   %拟合
CC(3, :) = fit_allan(sqrt(dt'), data_z', 2)';   %拟合

disp (CC);

Q = abs(CC(:, 1)) / sqrt(3);         %量化噪声，单位：arcsec
N = abs(CC(:, 2)) / 60;	            %角度随机游走，单位：deg/h^0.5
Bs = abs(CC(:, 3)) / 0.6643;	        %零偏不稳定性，单位：deg/h
K = abs(CC(:, 4)) * sqrt(3) * 60;	%角速率游走，单位：deg/h/h^0.5
R = abs(CC(:, 5)) * sqrt(2) * 3600;	%速率斜坡，单位：deg/h/h

fprintf('16448\n');
fprintf('量化噪声      X轴：%f Y轴：%f Z轴：%f  单位：arcsec\n', Q(1), Q(2), Q(3));
fprintf('角度随机游走  X轴：%f Y轴：%f Z轴：%f  单位：deg/h^0.5\n', N(1), N(2), N(3));
fprintf('零偏不稳定性  X轴：%f Y轴：%f Z轴：%f  单位：deg/h\n', Bs(1), Bs(2), Bs(3));
fprintf('角速率游走    X轴：%f Y轴：%f Z轴：%f  单位：deg/h/h^0.5\n', K(1), K(2), K(3));
fprintf('速率斜坡      X轴：%f Y轴：%f Z轴：%f  单位：deg/h/h\n', R(1), R(2), R(3));
fprintf('---------------------------------------\n');

DD(:, 1) = CC(1, 1)*sqrt(dt).^(-2) + CC(1, 2)*sqrt(dt).^(-1) + CC(1, 3)*sqrt(dt).^(0) + CC(1, 4)*sqrt(dt).^(1) + CC(1, 5)*sqrt(dt).^(2);    %生成拟合函数
DD(:, 2) = CC(2, 1)*sqrt(dt).^(-2) + CC(2, 2)*sqrt(dt).^(-1) + CC(2, 3)*sqrt(dt).^(0) + CC(2, 4)*sqrt(dt).^(1) + CC(2, 5)*sqrt(dt).^(2);    %生成拟合函数
DD(:, 3) = CC(3, 1)*sqrt(dt).^(-2) + CC(3, 2)*sqrt(dt).^(-1) + CC(3, 3)*sqrt(dt).^(0) + CC(3, 4)*sqrt(dt).^(1) + CC(3, 5)*sqrt(dt).^(2);    %生成拟合函数

loglog(dt, DD);   %画双对数坐标图
