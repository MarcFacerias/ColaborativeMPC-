% Test Observer
clear all 
% Add paths of needed functions
path = genpath('/home/marc/IRI_Internship/tfm/matlabCode/Codi Obs Matlab');
addpath(path);

pathSedumi = genpath('/home/marc/Matlab/Sedumi');
addpath(pathSedumi);

pathYalmip = genpath('/home/marc/Matlab/yalmip');
addpath(pathYalmip)

pathZonotopes = genpath('/home/marc/IRI_Internship/tfm/matlabCode/Simulation Systol2019');
addpath(pathZonotopes)

%%

%Constant definition for the RC car 
lf          = 0.125;
lr          = 0.125;
m           = 1.98;
I           = 0.03;
Cf          = 60;
Cr          = 60;  
mu          = 0.1;

g           = 9.8;
dt          = 0.005;

constants = [lf, lr, m, I, Cf, Cr, mu, g, dt];

dt = 0.005;
N = 500;

x_ant = zeros(6,1);
x_ant(1) = 1.0112;
x_ant(2) = 0.0115;
x_ant(3) = 0.0946;

y_ant = zeros(5,1);
y_ant(1) = 1.0112;
y_ant(2) = 0.0946;

cxio = x_ant;
Rxio = eye(6);
n_red = 6;
xmax = [];
xmin = [];
center = [];
x = x_ant;
ax_old = 0;
ay_old = 0;

C       = [ 1 0 0 0 0 0; 
            0 0 1 0 0 0;
            0 0 0 1 0 0;
            0 0 0 0 1 0;
            0 0 0 0 0 1];   
               
Ew   = diag([0.01  0.02  0.02   0.02  0.01  0.01]);  	% [ vx, vy, wz, x, y, theta ]
Ev   = diag([ 0.05 0.03  0.25  0.25  0.25 ]);            % measured variables

MAIN_ObserverUIO(); %computar obs
u = [0.25,1.0]';
dist_estimation = [0.0;0.0;0.0];

               
%%
a = -0.25;
b = 0.25;
noiseSignal = a + (b-a).*rand(1,N);

for i=2:N
    
   steer = u(1);
   accel = u(2);
   
   for j=1:10

       T = dt*j : dt/5 : dt*j+dt;
       [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ steer; accel ], constants, Ew), T, x_ant);  

       x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)];

       y = C * x_sim + (noiseSignal(i) * [ 0.05 0.03  0.25  0.25  0.25 ]');

       vx = cxio(1);
       vy = cxio(2);
       theta = cxio(6);
       steer = u(1);
       
       dy = (y-y_ant)/dt;
       
       E   = [-mu,0,0,0,0,0]'; 
       tau = pinv(C*E);

       [ L ] = K_Computation( steer, vx, vy, theta, SchedVars_Limits, Llmi_IOA ); 

       [ A_obs, B_obs, A, B, ~, ~ ] = Continuous_AB_Comp( steer, ...
           vx, vy, theta, C, 'Varianttt B' );
        L_obs = -dt*L;
        
        Ad = eye(6) + A*dt;
        Bd = B*dt;
        % disturbance estimatimation
         dist_estimation_max = tau*(y - C*(Ad*cxio + Bd*u ));
        dist_estimation_min = tau*(dy - C*(A*cxio + B*u) + [ 0.05 0.03  0.25  0.25  0.25 ]');
  
        % center    
        cxxio = (A_obs - L_obs*C)*cxio + B_obs*u + L_obs*y_ant + E*tau*y;
  
        cxio  = cxxio;
        x_ant = x_sim;
        y_ant = y;
       
   end
   
   for j=1:6
       
       center(i,j)  = cxio(j);
       
   end
  
    x_hist(i,:) = x_sim + noiseSignal(i) * [ 0.05 0 0.03  0.25  0.25  0.25 ]';
    hist_dist_max(i,:) = dist_estimation_max';
    hist_dist_min(i,:) = dist_estimation_min';
    dy_hist(i,:) = dy;

end

 N_exp = N;
 figure, set(gcf,'DefaultLineLineWidth',2.5);
 sgtitle('IOA with UIO Estimator')
 for i=1:6
 
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     % IOA
      plot(xmax(:,i));
      plot(x_hist(1:N_exp,i),'b');
      plot(center(1:N_exp,i),'r');
      
      plot(xmin(:,i));

     hold off
     legend('max','simulator','center','xmin')
%      legend('simulator','center')
     grid on
%      rmse(i)    = sqrt(sum((center(:,i) - x_hist(1:N,i)).^2)/N);
    
 end
 figure();
 hold on
%  plot(hist_dist_max(1:N_exp,1))
 plot(hist_dist_min(1:N_exp,1))
 hold off 
 legend('d1')
 grid on 
 sgtitle('disturbance estimation')
%  figure();
%  lab = categorical({'vx','vy','w','x','y','theta'});
% %  bar(lab,rmse)
%  legend('IOA')
 
 
 function E = update_E (alpha, vw)

 %kia niro 

    lf  = 1.52;
    lr  = 1.22;
    m   = 1554;
    I   = 2200;
    g   = 9.81;
    Cd  = 0.29;
    Al  = 6.6849;
    ro_aire  = 1.225;
    mu          = 0.1;
%     E = [ 0                                        , -g*sin(alpha)/alpha ... 
%         
%           -1/m * 1/2 * ro_aire * Cd * Al * vw       ,   0 ;               ...
%          
%           -(lf-lr)/I * 1/2 * ro_aire * Cd * Al *vw ,   0;                ...
%         
%           0                                        ,   0;                ...
%           0                                        ,   0;                ...
%           0                                        ,   0;                ];

%     E = [ -mu         ,  0  ; ... 
%         
%           0       ,   -1/m ;                ...
%          
%           0 ,   -(lf-lr)/I ;                ...
%         
%           0         ,   0 ;                ...
%           0         ,   0 ;                ...
%           0         ,   0 ;                  ];
      
%   E = -1*[ 0         ,   1 , 1; ... 
% 
%        1          ,   0 ,0;                ...
% 
%        0          ,   1 ,0;                ...
% 
%       0           ,   0 ,0;                ...
%       0           ,   0 ,0;                ...
%       0           ,   0 ,0;                  ];
%   
%     E = -1*[ 1         ,   0; ... 
% 
%        0          ,   1;                ...
% 
%        0          ,   1;                ...
% 
%       0           ,   0;                ...
%       0           ,   0;                ...
%       0           ,   0;                  ];

E = [-0.1,0,0,0,0,0]';
        
 end
 
