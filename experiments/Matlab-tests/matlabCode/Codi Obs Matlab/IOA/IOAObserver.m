% Compute Observer
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


MAIN_Observer(); %computar obs


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

% defining temporal behaviour
dt = 0.005;
N = 500;

% initial conditions that avoid singularity at vx = 0

x_ant = zeros(6,1);
x_ant(1) = 1.0112;
x_ant(2) = 0.0115;
x_ant(3) = 0.0946;


y_ant = zeros(5,1);
y_ant(1) = 1.0112;
y_ant(2) = 0.0946;

% Initialization 
cxio = x_ant;
Rxio = eye(6);
n_red = 6;
xmax = [];
xmin = [];
center = [];
x = x_ant;

C       = [ 1 0 0 0 0 0; 
            0 0 1 0 0 0;
            0 0 0 1 0 0;
            0 0 0 0 1 0;
            0 0 0 0 0 1];   
               
Ew   = diag([ 0.05   0.03   0.03  0.25  0.25  0.25 ]);  	% [ vx, vy, wz, x, y, theta ]
Ev   = diag([ 0.01   0.02   0.02  0.01  0.01 ]);              % measured variables
u = [0.25,1.0]';
%%

% Update control actions -> control loop
a = -0.25;
b = 0.25;
noiseSignal = a + (b-a).*rand(1,N);

for i=2:N

  
   steer = u(1);
   accel = u(2);
   
   for j=1:10
       
       % Execute estimation loop 10 times for each control loop to ensure
       % steady state of the observer

       T = dt*j : dt/5 : dt*j+dt;

       [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ steer; accel ],constants), T, x_ant);  

       x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)] + (noiseSignal(i) * [0.05   0.03   0.03  0.25  0.25  0.25]');

       y = C * x_sim + (noiseSignal(i) * [0.01   0.02   0.02  0.01  0.01]');

       % Update LPV model and generate new gains
       vx = cxio(1);
       vy = cxio(2);
       theta = cxio(6);
       steer = u(1);

       [ L ] = K_Computation( steer, vx, vy, theta, SchedVars_Limits, Llmi ); 

       [ ~, ~, A, B, ~, ~ ] = Continuous_AB_Comp( steer, ...
           vx, vy, theta, C, 'Varianttt B' );
       % discretize matrices
       A_obs = eye(6) + dt*A;
       B_obs = dt*B;
       L_obs = -dt*L; % we need to change the sign due to the formulation of LMIS

       % center    
       xeio  = A_obs*cxio + B_obs*u;
       cxxio = xeio + (L_obs*(y_ant-(C*xeio))); % 2.7a
       %eixos   
       shape_priori = [A_obs*Rxio, Ew];
       Rxxio = [(eye(6) - L_obs*C)*shape_priori,- L_obs*Ev];
       
       %Update matrices
       cxio = cxxio;
       Rxio = Rxxio;
       x_ant = x_sim;
       y_ant = y;
       
       
   end
   
   for j=1:6
       
       Rxio_red = reduction(Rxxio,n_red);
       Exio     = envbox(Rxio);
       xmax(i,j)    = cxio(j) + abs(Exio (j,j));
       xmin(i,j)    = cxio(j) - abs(Exio (j,j));
       center(i,j)  = cxio(j);
       
   end
  
    x_hist(i,:) = x_sim;
    
end

 
 figure, set(gcf,'DefaultLineLineWidth',2.5);
 sgtitle('IOA Estimator')
 for i=1:6
 
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     % IOA
      plot(xmax(:,i));
      plot(x_hist(:,i),'b');
      plot(center(:,i),'r');
      plot(xmin(:,i));

     hold off
%      legend('simulator','center')
     legend('max','simulator','center','min')
     grid on
     rmse(i)    = sqrt(sum((center(:,i) - x_hist(1:N,i)).^2)/N);
    
 end
 figure();
 lab = categorical({'vx','vy','w','x','y','theta'});
 bar(lab,rmse)
 legend('IOA')
 
 
 
 
