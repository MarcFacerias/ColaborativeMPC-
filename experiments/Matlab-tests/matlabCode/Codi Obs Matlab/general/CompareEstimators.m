clear all
tic
%% Compare different estimators %%

path = genpath('/home/marc/IRI_Internship/tfm/matlabCode/Codi Obs Matlab');
addpath(path);

pathSedumi = genpath('/home/marc/Matlab/Sedumi');
addpath(pathSedumi);

pathYalmip = genpath('/home/marc/Matlab/yalmip');
addpath(pathYalmip)

pathZonotopes = genpath('/home/marc/IRI_Internship/tfm/matlabCode/Simulation Systol2019');
addpath(pathZonotopes)

MAIN_Observer(); %computar obs
MAIN_ObserverUIO(); %computar obs

syms vx vy w x y theta a steer 

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
N           = 500;
n_red       = 6;

constants = [lf, lr, m, I, Cf, Cr, mu, g, dt];

%Initialization 

x_ant = zeros(6,1);
x_ant(1) = 1.0112;
x_ant(2) = 0.0115;
x_ant(3) = 0.0946;
 
y_ant = zeros(5,1);
y_ant(1) = 1.0112;
y_ant(2) = 0.0946;

% Definitions of KF and matrices parameters
Ew   = [0.01  0.02  0.02   0.02  0.01  0.01];  	% [ vx, vy, wz, x, y, theta ]
Q  = diag(Ew); % covariance matrix of the perturbations
Ew = Q;

Ev   = [ 0.05 0.03  0.25  0.25  0.25 ];            % measured variables
R = diag(Ev); % covariance matrix of the measurement noise
Ev = R;

% Eq of the sys without modelling the drag force (Used to compute KF Jacobians)
% a_F = atan((vy + lf*w)/vx) - steer;
% a_R = atan((vy - lr*w)/vx);
% 
% FyF = -Cf * a_F;
% FyR = -Cr * a_R;
% 
% dx1 = a - FyF*sin(steer)/m  +  w*vy; 
% dx2 = ( FyF*cos(steer) + FyR ) / m  -  w*vx ;
% dx3 = ( FyF*lf*cos(steer) - FyR*lr  ) / I;
% dx4 = vx*cos(theta) - vy*sin(theta);
% dx5 = vx*sin(theta) + vy*cos(theta);
% dx6 = w;

%NEW MODEL -> USED BY EUGE
dx1 = a + w*(vy + (Cf*lf*sin(steer))/(m*vx)) - (Cf*steer*sin(steer))/m + (Cf*vy*sin(steer))/(m*vx);
dx2 = (Cf*steer*cos(steer))/m - (vy*(Cr + Cf*cos(steer)))/(m*vx) - w*(vx - (Cr*lr - Cf*lf*cos(steer))/(m*vx));
dx3 = (vy*(Cr*lr - Cf*lf*cos(steer)))/(I*vx) - (w*(Cf*cos(steer)*lf^2 + Cr*lr^2))/(I*vx) + (Cf*steer*lf*cos(steer))/I;
dx4 = vx*cos(theta) - vy*sin(theta);
dx5 = vy*cos(theta) + vx*sin(theta);
dx6 = w; 

% evaluate in x where we are doing the linearization 
tic
Jb = simplify(jacobian([dx1, dx2, dx3, dx4, dx5, dx6],[vx; vy; w; x; y; theta; steer; a]));
toc

% we obtain a 6x8 matrix -> first 6 columns A, 2 last columns B  this leads
% to f(x) = f(x0) + A(x0)*(x-x0) + B(u0)*(u-u0)

A_sim = Jb(:,1:6);
B_sim = Jb(:,7:8);

% Definition of C 

C     = [ 1 0 0 0 0 0; %equivalent to Hx
          0 0 1 0 0 0;
          0 0 0 1 0 0;
          0 0 0 0 1 0;
          0 0 0 0 0 1];
      
% Function that updates E if needed (might be LPV)
E     = [ -mu, 0, 0, 0 ,0 ,0 ]';
      
% initialization
states_antEKF = zeros(6,1);
states_antEKF(1) = 1.0112;
states_antEKF(2) = 0.0115;
states_antEKF(3) = 0.0946;

sim_states_ant    = states_antEKF;
states_antEKF_UIO = states_antEKF;
sim_true_ant      = states_antEKF;

y_meas    = zeros(5,1);
y_meas(1) = 1.0112;
y_meas(3) = 0.0946;
u         = [0.25, 1.0]';
u_ant     = u;

cxioIOA   = states_antEKF;
RxioIOA   = eye(6);
xmaxIOA   = [];
xminIOA   = [];
centerIOA = [];

cxioIOA_UIO   = states_antEKF;
RxioIOA_UIO   = eye(6);
xmaxIOA_UIO   = [];
xminIOA_UIO   = [];
centerIOA_UIO = [];


% offline noise definition
a = -1.0;
b = 1.0;
noiseSignal = a + (b-a).*rand(1,N);

for i=2:N
   
   % Update control actions -> control loop
   steer = u(1);
   accel = u(2);

   for j=1:10
       
       % Execute estimation loop 10 times for each control loop to ensure
       % steady state of the observer
       T = 0 : dt/5 : dt;
       [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ u_ant(1); u_ant(2) ], constants), T, sim_states_ant);  

       x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)]+ ([0.01  0.02  0.02   0.02  0.01  0.01]' * noiseSignal(i));
       y_sim    = C*x_sim + (noiseSignal(i) * [ 0.05 0.03  0.25  0.25  0.25 ]');
       x_true   = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)];
       
       %% EKF %%
       % Parse the states and update Jacobians
       vx      = states_antEKF(1); 
       vy      = states_antEKF(2); 
       w       = states_antEKF(3);
       x       = states_antEKF(4);
       y       = states_antEKF(5);
       theta   = states_antEKF(6);
       
       % instantiate partial derivatives
       A       = double(simplify(subs(A_sim)));
       B       = double(simplify(subs(B_sim)));
       
       % Compute gamma (O or tau in bibliography)
       gamma   = pinv(C*E);
       
       %Linearizing system
       Ad = eye(6) + A*dt;
       Bd = B*dt;
       [K, ~, ~] = dlqr(Ad', C', Q, R); % we can use lqr design due to observer-control duality
       
       x_pred  = states_antEKF + nl_sys(states_antEKF, u_ant, constants) * dt; % prediction step using non-linear model 
       
       % update step using steady state Kalman gain 
       e = y_meas - C*x_pred;
       statesEKF = x_pred + K'*e;
       
       %% EKF_UIO %%
       % Parse the states and update Jacobians
       vx      = states_antEKF_UIO(1); 
       vy      = states_antEKF_UIO(2); 
       w       = states_antEKF_UIO(3);
       x       = states_antEKF_UIO(4);
       y       = states_antEKF_UIO(5);
       theta   = states_antEKF_UIO(6);
       
       % instantiate partial derivatives
       A       = double(simplify(subs(A_sim)));
       B       = double(simplify(subs(B_sim)));
       E     = [ -mu, 0, 0, 0 ,0 ,0 ]';
       
       Ad = eye(6) + A*dt;
       Bd = B*dt;
       
       %Redifinig matrices for the UIO aproach 
       Azero_d = (eye(6) - E*gamma*C) * Ad ;
       Bzero_d = (eye(6) - E*gamma*C) * Bd ; 
       
       %Compute gain with modified matrices
       [K, ~, ~] = dlqr(Azero_d', C', Q, R);
       
       % Estimation of the states
       statesEKF_UIO = (eye(6) - E*gamma*C)*(states_antEKF_UIO + nl_sys(states_antEKF_UIO,u_ant,constants)*dt) + K'*y_meas - K'*C*states_antEKF_UIO + E*gamma*y_sim;
       
       %% IOA %%
       
       % Update LPV model and generate new gains
       vx = cxioIOA(1);
       vy = cxioIOA(2);
       theta = cxioIOA(6);
       
       [ L ] = K_Computation( steer, vx, vy, theta, SchedVars_Limits, Llmi ); 

       [ ~, ~, A, B, ~, ~ ] = Continuous_AB_Comp( steer, ...
           vx, vy, theta, C, 'Varianttt B' );
       % discretize matrices
       A_obs = eye(6) + dt*A;
       B_obs = dt*B;
       L_obs = -dt*L; % we need to change the sign due to the formulation of LMIS
       
       % center    
       xeioIOA  = A_obs*cxioIOA + B_obs*u;
       cxxioIOA = xeioIOA + (L_obs*(y_meas-(C*xeioIOA))); % 2.7a
       %eixos   
       shape_priori = [A_obs*RxioIOA, Ew];
       RxxioIOA = [(eye(6) - L_obs*C)*shape_priori,- L_obs*Ev];
       
       %% IOA_UIO %%
       
       vx = cxioIOA_UIO(1);
       vy = cxioIOA_UIO(2);
       theta = cxioIOA_UIO(6);
       E     = [-mu*3,0,0,0,0,0]';
       [ L ] = K_Computation( steer, vx, vy, theta, SchedVars_Limits, Llmi_IOA );
       gamma = pinv(C*E);
       
       [ A_obs, B_obs, A, B, ~, ~ ] = Continuous_AB_Comp( steer, ...
           vx, vy, theta, C, 'Varianttt B' );
       L_obs = -dt*L;
       
       % center    
       cxxioIOA_UIO = (A_obs - L_obs*C)*cxioIOA_UIO + B_obs*u + L_obs*y_meas + E*gamma*y_sim;
       
       %eixos   
       RxxioIOA_UIO = [(A_obs - L_obs*C)*RxioIOA_UIO,(eye(6) - E*gamma*C)*Ew, (-E*gamma - L_obs)*Ev];
       
       
       %update constants for next iteration
       states_antEKF     = statesEKF;
       states_antEKF_UIO = statesEKF_UIO;
       sim_states_ant    = x_sim;
       sim_true_ant      = x_true;
       cxioIOA           = cxxioIOA;
       RxioIOA           = RxxioIOA;
       cxioIOA_UIO       = cxxioIOA_UIO;
       RxioIOA_UIO       = RxxioIOA_UIO;
       y_meas            = y_sim;
       u_ant             = [steer;accel];
   end
       
  
  for j=1:6
       
       Rxio_red            = reduction(RxxioIOA_UIO,n_red);
       Exio                = envbox(RxioIOA_UIO);
       xmaxIOA_UIO(i,j)    = cxioIOA_UIO(j) + abs(Exio (j,j));
       xminIOA_UIO(i,j)    = cxioIOA_UIO(j) - abs(Exio (j,j));
       centerIOA_UIO(i,j)  = cxioIOA_UIO(j);
       
  end
  
  for j=1:6
       
       Rxio_red        = reduction(RxxioIOA,n_red);
       Exio            = envbox(RxioIOA);
       xmaxIOA(i,j)    = cxioIOA(j) + abs(Exio (j,j));
       xminIOA(i,j)    = cxioIOA(j) - abs(Exio (j,j));
       centerIOA(i,j)  = cxioIOA(j);
       
  end
   
   %Saving historic of values
   x_hist(i,:)            = x_sim';
   estateEKF_hist(i,:)    = statesEKF_UIO';
   estateEKF_UIOhist(i,:) = statesEKF';
   hist_e(i,:)            = e;
   x_hist_true(i,:)       =x_true';
   

end
toc
figure;
sgtitle('Comparison of centers')
for i=1:6
          
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     plot(x_hist(:,i));
     plot(estateEKF_hist(:,i));
     plot(estateEKF_UIOhist(:,i));
     plot(centerIOA(:,i));
     plot(centerIOA_UIO(:,i));
                             
     legend('real','EKF','EKF_{UIO}', 'Cx_{IOA}', 'Cx_{IOA UIO}');
%      legend('real','Cx_{IOA}', 'Cx_{IOA UIO}');
     rmse_IOA(i)        = sqrt(sum((centerIOA(:,i) - x_hist(1:N,i)).^2)/N);
     rmse_IOA_UIO(i)    = sqrt(sum((centerIOA_UIO(:,i) - x_hist(1:N,i)).^2)/N);
     rmse_EKF(i)        = sqrt(sum((estateEKF_hist(:,i) - x_hist(1:N,i)).^2)/N);
     rmse_EKF_UIO(i)    = sqrt(sum((estateEKF_UIOhist(:,i) - x_hist(1:N,i)).^2)/N);

     hold off
     grid on 
    
end

figure;
sgtitle('Comparison of centers and uncertainty regions')

for i=1:6
          
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     plot(xmaxIOA_UIO(:,i));
     plot(xmaxIOA(:,i));
     plot(x_hist(:,i));
%      plot(estateEKF_hist(:,i));
%      plot(estateEKF_UIOhist(:,i));
     plot(centerIOA(:,i));
     plot(centerIOA_UIO(:,i));
     plot(xminIOA(:,i));
     plot(xminIOA_UIO(:,i));
     
%      legend('xmaxIOA_UIO', 'xmaxIOA','real','EKF','EKF_{UIO}', 'Cx_{IOA}', 'Cx_{IOA UIO}','xminIOA', 'xminIOA_UIO');
%      legend('xmaxIOA_UIO', 'xmaxIOA','xminIOA', 'xminIOA_UIO');
%      legend('xmaxIOA_UIO', 'xmaxIOA','real', 'Cx_{IOA}', 'Cx_{IOA UIO}','xminIOA', 'xminIOA_UIO');

     hold off
     
     grid on 
    
end
 
 figure();
 lab = categorical({'vx','vy','w','x','y','theta'});
 hold on 
 bar(lab,rmse_IOA)
 bar(lab,rmse_IOA_UIO)
 bar(lab,rmse_EKF)
 bar(lab,rmse_EKF_UIO)
 legend('IOA')
 
 sgtitle('Trajectories of each estimation')
 figure();
 grid on
 hold on 
     plot(x_hist(:,4),x_hist(:,5));
     plot(estateEKF_hist(:,4),estateEKF_hist(:,5));
     plot(estateEKF_UIOhist(:,4),estateEKF_UIOhist(:,5));
     plot(centerIOA(:,4),centerIOA(:,5));
     plot(centerIOA_UIO(:,4),centerIOA_UIO(:,5));
     legend('real','EKF','EKF_{UIO}', 'Cx_{IOA}', 'Cx_{IOA UIO}');
hold off



function dx = nl_sys(states,u,constants)
    % eq of the system (model)
    lf          = constants(1);
    lr          = constants(2);
    m           = constants(3);
    I           = constants(4);
    Cf          = constants(5);
    Cr          = constants(6);
    mu          = constants(7);
    g           = constants(8);
    dt          = constants(9);

    vx          = states(1); 
    vy          = states(2); 
    w           = states(3);
    x           = states(4);
    y           = states(5);
    theta       = states(6);
    
    a           = u(2);
    steer       = u(1);
    
%     a_F = atan((vy + lf*w)/vx) - delta;
%     a_R = atan((vy - lr*w)/vx);
% 
%     FyF = -Cf * a_F;
%     FyR = -Cr * a_R;
%     
%     F_drag = 0;
% 
%     dx(1,1) = a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
%     dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
%     dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
%     dx(4,1) = vx*cos(theta) - vy*sin(theta);
%     dx(5,1) = vx*sin(theta) + vy*cos(theta);
%     dx(6,1) = w;

dx(1,1) = a + w*(vy + (Cf*lf*sin(steer))/(m*vx)) - (Cf*steer*sin(steer))/m + (Cf*vy*sin(steer))/(m*vx);
dx(2,1)= (Cf*steer*cos(steer))/m - (vy*(Cr + Cf*cos(steer)))/(m*vx) - w*(vx - (Cr*lr - Cf*lf*cos(steer))/(m*vx));
dx(3,1) = (vy*(Cr*lr - Cf*lf*cos(steer)))/(I*vx) - (w*(Cf*cos(steer)*lf^2 + Cr*lr^2))/(I*vx) + (Cf*steer*lf*cos(steer))/I;
dx(4,1) = vx*cos(theta) - vy*sin(theta);
dx(5,1) = vy*cos(theta) + vx*sin(theta);
dx(6,1) = w; 
end

