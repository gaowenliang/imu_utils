clear 
close all

dt = dlmread('data/data_gx4_t.txt');         
data_x = dlmread('data/data_gx4_x.txt'); 
data_y= dlmread('data/data_gx4_y.txt'); 
data_z = dlmread('data/data_gx4_z.txt'); 
data_draw=[data_x data_y data_z] /3600;

data_sim_x= dlmread('data/data_gx4_sim_x.txt'); 
data_sim_y= dlmread('data/data_gx4_sim_y.txt'); 
data_sim_z= dlmread('data/data_gx4_sim_z.txt'); 
data_sim_draw=[data_sim_x data_sim_y data_sim_z] /3600;


figure
loglog(dt, data_draw , 'o');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
% ylabel('Sigma:deg/h');             
legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , '-');
