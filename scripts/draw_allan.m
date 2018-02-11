clear 
close all

dt = dlmread('data/data_A3_t.txt');         %���ı��ж�ȡ��ݣ���λ��deg/s�����ʣ�100Hz
data_x = dlmread('data/data_A3_x.txt'); 
data_y= dlmread('data/data_A3_y.txt'); 
data_z = dlmread('data/data_A3_z.txt'); 
data_draw=[data_x  ] ;

data_sim_x= dlmread('data/data_A3_sim_x.txt'); 
data_sim_y= dlmread('data/data_A3_sim_y.txt'); 
data_sim_z= dlmread('data/data_A3_sim_z.txt'); 
data_sim_draw=[data_sim_x  ] ;


figure
loglog(dt, data_draw , '*');
xlabel('time:sec');                 %���x���ǩ
% ylabel('Sigma:deg/h');              %���y���ǩ
legend('x','y','z');       %��ӱ�ע
grid on;                            %��������
hold on;                            %ʹͼ�񲻱�����
loglog(dt, data_sim_draw , '-');

CC(1, :) = fit_allan(sqrt(dt'), data_x', 2)';   %���
CC(2, :) = fit_allan(sqrt(dt'), data_y', 2)';   %���
CC(3, :) = fit_allan(sqrt(dt'), data_z', 2)';   %���

disp (CC);

Q = abs(CC(:, 1)) / sqrt(3);         %��������λ��arcsec
N = abs(CC(:, 2)) / 60;	            %�Ƕ�������ߣ���λ��deg/h^0.5
Bs = abs(CC(:, 3)) / 0.6643;	        %��ƫ���ȶ��ԣ���λ��deg/h
K = abs(CC(:, 4)) * sqrt(3) * 60;	%���������ߣ���λ��deg/h/h^0.5
R = abs(CC(:, 5)) * sqrt(2) * 3600;	%����б�£���λ��deg/h/h

fprintf('A3\n');
fprintf('��������      X�᣺%f Y�᣺%f Z�᣺%f  ��λ��arcsec\n', Q(1), Q(2), Q(3));
fprintf('�Ƕ��������  X�᣺%f Y�᣺%f Z�᣺%f  ��λ��deg/h^0.5\n', N(1), N(2), N(3));
fprintf('��ƫ���ȶ���  X�᣺%f Y�᣺%f Z�᣺%f  ��λ��deg/h\n', Bs(1), Bs(2), Bs(3));
fprintf('����������    X�᣺%f Y�᣺%f Z�᣺%f  ��λ��deg/h/h^0.5\n', K(1), K(2), K(3));
fprintf('����б��      X�᣺%f Y�᣺%f Z�᣺%f  ��λ��deg/h/h\n', R(1), R(2), R(3));
fprintf('---------------------------------------\n');

DD(:, 1) = CC(1, 1)*sqrt(dt).^(-2) + CC(1, 2)*sqrt(dt).^(-1) + CC(1, 3)*sqrt(dt).^(0) + CC(1, 4)*sqrt(dt).^(1) + CC(1, 5)*sqrt(dt).^(2);    %�����Ϻ���
DD(:, 2) = CC(2, 1)*sqrt(dt).^(-2) + CC(2, 2)*sqrt(dt).^(-1) + CC(2, 3)*sqrt(dt).^(0) + CC(2, 4)*sqrt(dt).^(1) + CC(2, 5)*sqrt(dt).^(2);    %�����Ϻ���
DD(:, 3) = CC(3, 1)*sqrt(dt).^(-2) + CC(3, 2)*sqrt(dt).^(-1) + CC(3, 3)*sqrt(dt).^(0) + CC(3, 4)*sqrt(dt).^(1) + CC(3, 5)*sqrt(dt).^(2);    %�����Ϻ���

% loglog(dt, DD);   %��˫�������ͼ
