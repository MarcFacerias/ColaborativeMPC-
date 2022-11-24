   
clear all 

    x_ant = zeros(6,1);
    x_ant(1) = 1.0112;
    x_ant(2) = 0.0115;
    x_ant(3) = 0.0946;
    dt = 0.005;

    
   steer = 0;
   accel = 2.0;
   for i=1:1000
     

           T = 0:dt/10:dt;
%            T = dt*i : dt/5 : dt*i+dt;

           [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ 0.1; 1.2 ]), T, x_ant);  
           x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)] + noiseSignal(index) * [0.05 0.05 0.05 0.05 0.05 0.2]';


           x_ant = x_sim;

       
       x_hist(i,:) = x_sim';
   end
   
   
for i=1:6
 
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     plot(x_hist(:,i),'b');

     hold off
     
     grid on 
    
 end
   