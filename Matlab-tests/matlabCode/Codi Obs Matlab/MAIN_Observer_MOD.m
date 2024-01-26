%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autor: Eugenio Alcala Baselga
% Date: 25-October-2019
% Place: CS2AC-UPC-IRI
%
% The aim is to create an estimator for IDIADA project
% 
% Que podemos medir?
% - Velocidad lineal (vx) con encoders
% - Velocidad angular (w) con IMU
% - Posicion (x,y) con GPS
% - Orientacion (theta) con la IMU (aunque esta no suele ser muy fiable)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definitions and initialization:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% 3 scheduling variables: [vx, vy, delta]

% System definition:
n_meas        = 5; % num. measured states
n_s           = 6; % num. system states
n_sched_vars  = 4; % num. schedulng variables 

% % Low velocities:
% SchedVars_Limits = [ 0.1       1.1;   % vx
%                     -0.5     0.5;   % vy
%                     -2         2;   % w
%                     -0.3     0.3;   % steering
%                     -8         8;   % acceleration
%                     -pi/2  pi/2];   % theta
                
% Higher velocities:                
SchedVars_Limits = [ 1        14;    % vx
                    -2         2;   % vy
                    -3         3;   % w
                    -0.3     0.3;   % steering
                    -8         8;   % acceleration
                    -10      10];   % theta                

steer_vec   = SchedVars_Limits(4,:)';
vx_vec      = SchedVars_Limits(1,:)';
vy_vec      = SchedVars_Limits(2,:)';
theta_vec   = SchedVars_Limits(6,:)';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Diseño del observador:

Q   = cov(10^-10*diag([ 0.15   0.15   0.15  0.15  0.15  0.15 ]));  	% [ vx, vy, wz, x, y, theta ]
R   = cov(10^-10*diag([ 0.2    0.2    0.2    0.2  0.2 ]));              % measured variables

A_obs       = zeros(6, 6, 2^n_sched_vars);   
C_obs       = [ 1 0 0 0 0 0; 
                0 0 1 0 0 0;
                0 0 0 1 0 0;
                0 0 0 0 1 0;
                0 0 0 0 0 1];         

index   = 1;
for l=1:2
    Vx = vx_vec(l);
    for i=1:2
        Vy = vy_vec(i);
        for j=1:2
            Steer = steer_vec(j);
            for k=1:2
                Theta = theta_vec(k);
            
                [ A_obs(:,:,index), B_obs, A, B, Ef, Opseudinv ] = Continuous_AB_Comp( Steer, Vx, Vy, Theta, C_obs, 'Invariant B' );
            
                index = index + 1;
            end
        end
    end 
end

% IS THE SYSTEM OBSERVABLE ??
for i=1:2^n_sched_vars
    if(rank( obsv(A_obs(:,:,i), C_obs) ) == length(A_obs(:,:,i)))
        disp('Observable')
    end
end


%%
Wlmi        = sdpvar(n_s, n_meas, 2^(n_sched_vars));
LMI1        = sdpvar(n_s+n_s+n_meas, n_s+n_s+n_meas, 2^(n_sched_vars));
Y           = sdpvar(n_s,n_s); 
gamma       = 1; 
alphaObs    = 0.0;
Llmi        = zeros(n_s, n_meas, 2^(n_sched_vars));
CL_Poles    = zeros(n_s, 1, 2^(n_sched_vars));  

F    = [gamma*eye(n_s) eye(n_s); eye(n_s) Y] >= 0;

for i=1:2^(n_sched_vars)                      
               
    LMI1(:,:,i) = [Y*A_obs(:,:,i)+Wlmi(:,:,i)*C_obs+A_obs(:,:,i)'*Y+C_obs'*Wlmi(:,:,i)'+Y*2*(alphaObs) , Y*(Q^0.5)' , Wlmi(:,:,i) ;
                Y*Q^0.5                                                             , -eye(n_s)    ,zeros(n_s,n_meas);
                Wlmi(:,:,i)'                                                         , zeros(n_meas,n_s) ,inv(-R)  ];               

    F = F + [LMI1(:,:,i)<=0];
end            

ops = sdpsettings('warning',1,'verbose',1,'solver','sedumi','cachesolvers',1);
optimize(F, gamma, ops);

for i=1:2^(n_sched_vars)
    eig(value(Y));
    Llmi(:,:,i) =  inv(value(Y)) * value(Wlmi(:,:,i));
    eig(A_obs(:,:,i) + Llmi(:,:,i) * C_obs)
end




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VALIDACION:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% load 'References_Berkeley.mat'
% 
% steer = UU(1,:);
% accel = UU(2,:);
% 
% long_sim = length(accel);
% 
% x_est   = [ XX(1:3,1); 0; 0; 0 ];
% x0      = x_est;
% 
% STATES  = zeros(6,long_sim);
% 
% y_meas =  [x_est(1); x_est(3); x_est(4); x_est(5); x_est(6) ]; % Variables medidas: [vx w x y theta]
% 
% Ts = 0.03;
% N = 6;
% Ts_obs = 0.03/N;
% 
% noiseSignal = (rand(1, 428)-0.5)/10;
% 
% for index = 2:428 
% 
%     for i=1:N
%     
%         T = Ts_obs*i : Ts_obs/10 : Ts_obs*i+Ts_obs;
% 
%         [ T, x ] = ode45(@( t,x ) nonlinear_Berkeley_model( t, x, [ steer(index); accel(index) ]), T, x0);  
% 
%         x0      = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)] + noiseSignal(index) * [0.05 0.05 0.05 0.05 0.05 0.2]';
%   
%         y_meas = C_obs * x0 ;
%   
%         x_est = State_Estimation( x_est, y_meas, accel(index), steer(index), C_obs, index, Llmi, Ts_obs, SchedVars_Limits);    
%     
%     end 
% 
%     STATES(:,index) = x0;
%     XEST(:,index) = x_est(:,end); 
% 
% end
% 
% % err_vx = immse(STATES(1,1:index), XEST(1,1:index))
% % err_vy = immse(STATES(2,1:index), XEST(2,1:index))
% % err_w  = immse(STATES(3,1:index), XEST(3,1:index))
% 
% %%
% 
% figure(1), 
% subplot(6,1,1), plot(STATES(1,1:index),'r'), hold on, plot(XEST(1,1:index))
% subplot(6,1,2), plot(STATES(2,1:index),'r'), hold on, plot(XEST(2,1:index))
% subplot(6,1,3), plot(STATES(3,1:index),'r'), hold on, plot(XEST(3,1:index))
% subplot(6,1,4), plot(STATES(4,1:index),'r'), hold on, plot(XEST(4,1:index))
% subplot(6,1,5), plot(STATES(5,1:index),'r'), hold on, plot(XEST(5,1:index))
% subplot(6,1,6), plot(STATES(6,1:index),'r'), hold on, plot(XEST(6,1:index))
% 
% figure (2), plot(STATES(4,1:index), STATES(5,1:index), 'r')
% hold on, plot(XEST(4,1:index), XEST(5,1:index))
% legend('Real', 'Estimated')



