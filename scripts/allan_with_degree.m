clear 
close all

dt = dlmread('../data/data_gx4_gyr_t.txt');         
data_x = dlmread('../data/data_gx4_gyr_x.txt'); 
data_y= dlmread('../data/data_gx4_gyr_y.txt'); 
data_z = dlmread('../data/data_gx4_gyr_z.txt'); 
data_draw=(data_x+data_y+data_z)/3 /3600;

data_sim_x= dlmread('../data/data_gx4_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_gx4_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_gx4_sim_gyr_z.txt'); 
data_sim_draw=(data_sim_x+data_sim_y+data_sim_z)/3 /3600;

figure
loglog(dt, data_draw , 'r+');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
ylabel('Sigma:deg/s');             
% legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , 'r-');

dt = dlmread('../data/data_BMI160_gyr_t.txt');         
data_x = dlmread('../data/data_BMI160_gyr_x.txt'); 
data_y = dlmread('../data/data_BMI160_gyr_y.txt'); 
data_z = dlmread('../data/data_BMI160_gyr_z.txt'); 
data_draw=(data_x+data_y+data_z)/3 /3600;
data_sim_x= dlmread('../data/data_BMI160_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_BMI160_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_BMI160_sim_gyr_z.txt'); 
data_sim_draw=(data_sim_x+data_sim_y+data_sim_z)/3 /3600;
loglog(dt, data_draw , '-');
xlabel('time:sec');
loglog(dt, data_sim_draw , '-');


dt = dlmread('../data/data_16448_gyr_t.txt');         
data_x = dlmread('../data/data_16448_gyr_x.txt'); 
data_y = dlmread('../data/data_16448_gyr_y.txt'); 
data_z = dlmread('../data/data_16448_gyr_z.txt'); 
data_draw=(data_x+data_y+data_z)/3/3600;
data_sim_x= dlmread('../data/data_16448_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_16448_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_16448_sim_gyr_z.txt'); 
data_sim_draw=(data_sim_x+data_sim_y+data_sim_z)/3/3600;
loglog(dt, data_draw , 'b+');
xlabel('time:sec');                
loglog(dt, data_sim_draw , 'b-');

dt = dlmread('../data/data_A3_gyr_t.txt');         
data_x = dlmread('../data/data_A3_gyr_x.txt'); 
data_y = dlmread('../data/data_A3_gyr_y.txt'); 
data_z = dlmread('../data/data_A3_gyr_z.txt'); 
data_draw=(data_x+data_y+data_z)/3/3600;
data_sim_x= dlmread('../data/data_A3_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_A3_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_A3_sim_gyr_z.txt'); 
data_sim_draw=(data_sim_x+data_sim_y+data_sim_z)/3/3600;
loglog(dt, data_draw , 'g+');
xlabel('time:sec');                
loglog(dt, data_sim_draw , 'g-');

dt = dlmread('../data/data_N3_gyr_t.txt');         
data_x = dlmread('../data/data_N3_gyr_x.txt'); 
data_y = dlmread('../data/data_N3_gyr_y.txt'); 
data_z = dlmread('../data/data_N3_gyr_z.txt'); 
data_draw=(data_x+data_y+data_z)/3/3600;
data_sim_x= dlmread('../data/data_N3_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_N3_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_N3_sim_gyr_z.txt'); 
data_sim_draw=(data_sim_x+data_sim_y+data_sim_z)/3/3600;
loglog(dt, data_draw , 'k+');
xlabel('time:sec');                
loglog(dt, data_sim_draw , 'k-');