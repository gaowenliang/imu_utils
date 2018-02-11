clear 
close all

% dt = dlmread('data/data_16448_t.txt');         %���ı��ж�ȡ��ݣ���λ��deg/s�����ʣ�100Hz
% data_x = dlmread('data/data_16448_x.txt'); 
% data_y= dlmread('data/data_16448_y.txt'); 
% data_z = dlmread('data/data_16448_z.txt'); 
dt = dlmread('data/data_A3_t.txt');         %���ı��ж�ȡ��ݣ���λ��deg/s�����ʣ�100Hz
data_x = dlmread('data/data_A3_x.txt'); 
data_y= dlmread('data/data_A3_y.txt'); 
data_z = dlmread('data/data_A3_z.txt'); 
data_sim_y = dlmread('data/data_A3sim_z.txt'); 

data_draw=[data_x data_y data_z] ;
figure
loglog(dt, data_draw , '*');
xlabel('time:sec');                 %���x���ǩ
% ylabel('Sigma:deg/h');              %���y���ǩ
legend('x','y','z');       %��ӱ�ע
grid on;                            %��������
hold on;                            %ʹͼ�񲻱�����

CC(1, :) = fit_allan(dt', ( data_x.^2)', 2)';   %���
CC(2, :) = fit_allan(dt', ( data_y.^2)', 2)';   %���
CC(3, :) = fit_allan(dt', ( data_z.^2)', 2)';   %���

disp (CC);

Q  = sqrt( abs(CC(:, 1))) / sqrt(3);         %��������λ��arcsec
N  = sqrt( abs(CC(:, 2))) / 60;	            %�Ƕ�������ߣ���λ��deg/h^0.5
Bs = sqrt( abs(CC(:, 3))) / 0.6643;	        %��ƫ���ȶ��ԣ���λ��deg/h
K  = sqrt( abs(CC(:, 4))) * sqrt(3) * 60;	%���������ߣ���λ��deg/h/h^0.5
R  = sqrt( abs(CC(:, 5))) * sqrt(2) * 3600;	%����б�£���λ��deg/h/h

fprintf('A3\n');
fprintf('��������      X %f Y %f Z %f  ��λ��arcsec\n', Q(1), Q(2), Q(3));
fprintf('�Ƕ��������    X %f Y %f Z %f  ��λ��deg/h^0.5\n', N(1), N(2), N(3));
fprintf('��ƫ���ȶ���    X %f Y %f Z %f  ��λ��deg/h\n', Bs(1), Bs(2), Bs(3));
fprintf('����������    X %f Y %f Z %f  ��λ��deg/h/h^0.5\n', K(1), K(2), K(3));
fprintf('����б��       X %f Y %f Z %f  ��λ��deg/h/h\n', R(1), R(2), R(3));
fprintf('---------------------------------------\n');

DD(:, 1) = sqrt( CC(1, 1)*dt.^(-2) + CC(1, 2)*dt.^(-1) + CC(1, 3)*dt.^(0) + CC(1, 4)*dt.^(1) + CC(1, 5)*dt.^(2));
DD(:, 2) = sqrt( CC(2, 1)*dt.^(-2) + CC(2, 2)*dt.^(-1) + CC(2, 3)*dt.^(0) + CC(2, 4)*dt.^(1) + CC(2, 5)*dt.^(2));
DD(:, 3) = sqrt( CC(3, 1)*dt.^(-2) + CC(3, 2)*dt.^(-1) + CC(3, 3)*dt.^(0) + CC(3, 4)*dt.^(1) + CC(3, 5)*dt.^(2));

loglog(dt, DD);   %��˫�������ͼ
