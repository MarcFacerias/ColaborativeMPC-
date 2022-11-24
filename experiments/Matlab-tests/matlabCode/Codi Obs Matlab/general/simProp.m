% Test Observer
% clear all 
% data  =[states(6),acc,steer]
% initialization
% addpath '/home/marc/IRI_Internship/tfm/matlabCode/Codi Obs Matlab'
% MAIN_Observer_MOD(); %computar obs


data = readNPY("experimentObs.npy"); %cargar datos experimento
dataObs = readNPY("experimentObs.npy");
load("Estimator_Gains_HS.mat");
% data = vx,vy,psiDot,x,y,psi
dt = 0.005;
[N,~] = size(data);
y = data(2,[1,3,4,5,6])';
x_ant = data(2,1:6)'; 
y_ant = data(2,[1,3,4,5,6])';
cxio = x_ant;
Rxio = eye(6);
n_red = 7;
xmax = [];
xmin = [];
center = [];

Ew             = [ 0.01 0 0 0 0 0; 
                   0 0.01 0 0 0 0;
                   0 0 0.01 0 0 0;
                   0 0 0 0.01 0 0;
                   0 0 0 0 0.01 0;
                   0 0 0 0 0 0.01];
              
Ev             = [ 0.01 0 0 0 0 0; 
                   0 0.01 0 0 0 0;
                   0 0 0.01 0 0 0;
                   0 0 0 0.01 0 0;
                   0 0 0 0 0 0.01];

C              = [ 1 0 0 0 0 0; 
                   0 0 1 0 0 0;
                   0 0 0 1 0 0;
                   0 0 0 0 1 0;
                   0 0 0 0 0 1];
              
        
for i=3:N
    
    y          = data(i-1,[1,3,4,5,6]); 
    u          = data(i-1,[7,8]);
   
%     vx         = data(i-1,1);
%     vy         = data(i-1,2);
    steer      = data(i-1,8);
%     theta      = data(i-1,6);
    
    if(i<5)
        vx      = y(1);
        vy      = y(2);
        theta   = y(5);
    else
        vx      = data(i-1,1);
        vy      = data(i-1,2);
        theta   = data(i-1,6);
    end
  
%      vx         = y(1);
%      vy         = cxio(2);
%      steer      = data(i,8);
%      theta      = y(5);

   
   [ L ] = K_Computation( steer, vx, vy, theta, SchedVars_Limits, Llmi ); 
   
   [ ~, ~, A, B, ~, ~ ] = Continuous_AB_Comp( steer, ...
       vx, vy, theta, C, 'Varianttt B' );
   
   
   A_obs = eye(6) + dt*A;
   B_obs = dt*B;
   L_obs = dt*L;
   
%    x     = A_obs * x_ant + B_obs * u';
%    y     = C * x ;
   
   % center    
   xeio=A_obs*cxio + B_obs*u';
   cxxio = xeio - (L_obs*(y'-(C*xeio))); % 2.7a

   %eixos   
   
   shape_priori = [A*Rxio,Ew];
   Rxxio = [(eye(6) - L_obs*C)*shape_priori,- L_obs*Ev];
   
   Rxio_red = reduction(Rxxio,n_red);
   Exio    = envbox(Rxio);
   
   for j=1:6
       xmax(i,j)    = cxio(j) + abs(Exio (j,j));
       xmin(i,j)    = cxio(j) - abs(Exio (j,j));
       center(i,j)  = cxio(j);
   end
   
    cxio = cxxio;
    Rxio = Rxxio;
%     x(i,:) = x';
%     x_ant = x;
%     y_ant = y;
    
end

 figure, set(gcf,'DefaultLineLineWidth',2.5);
 
 for i=1:6
 
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     % IOA
%      plot(xmax(:,i),'r');
       plot(center(:,i));
%        plot(x(:,i));
       plot(data(:,i),'b')
%        plot(dataObs(1:639,i),'r')
%      plot(xmin(:,i),'b');

     hold off
     
     grid on 
    
 end
