clear 
close all

dt = dlmread('../data/data_A3_gyr_t.txt');   

data_x = dlmread('../data/data_A3_gyr_x.txt'); 
data_y= dlmread('../data/data_A3_gyr_y.txt'); 
data_z = dlmread('../data/data_A3_gyr_z.txt'); 
% data_draw=[data_x data_y data_z] /3600;
% data_draw=[data_x ]/3600;
data_avg = (data_x+data_y+data_z)/3/3600 /51.3;

data_sim_x= dlmread('../data/data_A3_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_A3_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_A3_sim_gyr_z.txt'); 
% data_sim_draw=[data_sim_x data_sim_y data_sim_z] /3600;
data_sim_draw=[data_sim_x ]/3600;
data_sim_avg = (data_sim_x+data_sim_y+data_sim_z)/3/3600 /51.3;

sim_x= 10.^( -0.5 * log(dt) - 4);
sim_x2= 10.^( 0.5 * log(dt) - 7);

for n=1: 93
    if(sim_x(n)<10.^(-7))
        sim_x(n) = 0;
    end
    if(sim_x(n)>10.^(-2))
        sim_x(n) = 0;
    end
    
    if(sim_x2(n)<10.^(-7))
        sim_x2(n) = 0;
    end
    if(sim_x2(n)>10.^(-2))
        sim_x2(n) = 0;
    end
end

figure
loglog(dt, data_avg , 'o');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
% ylabel('Sigma:deg/h');             
legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_avg , '-');

loglog(dt, sim_x , '-');
loglog(dt, sim_x2 , '-');
