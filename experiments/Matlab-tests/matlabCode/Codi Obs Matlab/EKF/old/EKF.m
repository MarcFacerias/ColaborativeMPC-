% % with parameters
clear all
% close all

syms vx vy w x y theta a delta 

lf          = 0.125;
lr          = 0.125;
m           = 1.98;
I           = 0.03;
Cf          = 60;
Cr          = 60;  
mu          = 1.0;

% lf          = 1.52;
% lr          = 1.22;
% m           = 1554;
% I           = 2200;
% Cf          = 59054;
% Cr          = 107123;  
% mu          = 0.5;
% g           = 9.81;

g           = 9.8;
dt          = 0.005;
N           = 50;

constants = [lf, lr, m, I, Cf, Cr, mu, g, dt];

% % without parameters
% syms vx vy psiDot x y psi a delta lf lr m I Cf Cr mu 

% % % noise symbolic variables
syms n1 n2 n3 n4 n5 n6

% Definitions of KF parame ters
% Ew = [0.1,0.2,0.3,0.4,0.5,.6]; % covariance of the perturbations
Ew = [0.05   0.03   0.03  0.25  0.25  0.25];
Q = diag(Ew); % covariance matrix of the perturbations

% Ev = [0.1,0.2,0.3,0.4,0.5]; %covariance of the measurement noise
Ev = [0.01   0.02   0.02  0.01  0.01];
R = diag(Ev); % covariance matrix of the measurement noise

% Eq of the sys
a_F = atan((vy + lf*w)/vx) - delta;
a_R = atan((vy - lr*w)/vx);

FyF = -Cf * a_F;
FyR = -Cr * a_R;


dx1 = a - FyF*sin(delta)/m  +  w*vy + n1*Ew(1); 
dx2 = ( FyF*cos(delta) + FyR ) / m  -  w*vx + n2*Ew(2);
dx3 = ( FyF*lf*cos(delta) - FyR*lr  ) / I   + n3*Ew(3);
dx4 = vx*cos(theta) - vy*sin(theta) + n4*Ew(4);
dx5 = vx*sin(theta) + vy*cos(theta) + n5*Ew(5);
dx6 = w + n6*Ew(6);

% evaluate in x where we are doing the linearization 
tic
Jb = simplify(jacobian([dx1, dx2, dx3, dx4, dx5, dx6],[vx; vy; w; x; y; theta; delta; a; ...
    n1; n2; n3; n4; n5; n6]));
toc

% we obtain a 6x8 matrix -> first 6 columns A, 2 last columns B  this leads
% to f(x) = f(x0) + A(x0)*(x-x0) + B(u0)*(u-u0)

A_sim = Jb(:,1:6);
B_sim = Jb(:,7:8);
% Fn    = Jb(:,9:14); Used in recursive EKF

C     = [ 1 0 0 0 0 0; %equivalent to Hx
          0 0 1 0 0 0;
          0 0 0 1 0 0;
          0 0 0 0 1 0;
          0 0 0 0 0 1];    

% initialization
states_ant = zeros(6,1);
states_ant(1) = 1.0112;
states_ant(2) = 0.0115;
states_ant(3) = 0.0946;
sim_states_ant = states_ant;

y_meas    = zeros(5,1);
y_meas(1) = 1.0112;
y_meas(3) = 0.0946;
u         = [0.25, 5.0]';
u_ant     = u;


% at each time instant we instantiate this model with x(k-1) and u(k-1) ->
% must be inside a for.
dist_estimation = 0.0;
noiseSignal = (rand(1, N)-0.5)/10;
for i=2:N
    
   steer = u(1);
   accel = u(2);
   
   
   for j=1:10

       tic
       if i >1 && i < 20
           dist = 10;
       else
           dist = 0;

       end
       T = 0 : dt/5 : dt;
       [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ steer; accel ], constants), T, sim_states_ant);  
%        [ T, x ] = ode45(@( t,x ) nonlinear_model_disturbed( t, x, [ steer; accel ],dist, constants), T, sim_states_ant);  

       x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)];
       
       vx      = states_ant(1); 
       vy      = states_ant(2); 
       w       = states_ant(3);
       x       = states_ant(4);
       y       = states_ant(5);
       theta   = states_ant(6);
       
       delta   = u_ant(1);
       a       = u_ant(2);

       A       = double(simplify(subs(A_sim)));
       B       = double(simplify(subs(B_sim)));
        
       alpha = 0;
       vw    = 0;
       E     = update_E(alpha, vw, constants);
       tau   = pinv(C*E);
       dy    = (C*x_sim - y_meas)/dt;       
       
       %Ricatti steady state solution
       Ad = eye(6) + A*dt;
       Bd = B*dt;
       [K, S, P] = dlqr(Ad', C', Q, R);

       x_pred  = states_ant + nl_sys(states_ant, u_ant, constants) * dt;
       dist_estimation = tau*(dy - C * nl_sys(states_ant, u_ant, constants))

       e = y_meas - C*x_pred;
       states = x_pred + K'*e - E*tau*dy;
       
       e = C * x_sim - C*states;
       states_ant      = states;
       sim_states_ant  = x_sim;
       y_meas          = C * x_sim;
       u_ant           = [steer;accel];

       toc
       i
   end
   
   x_hist(i,:)     = x_sim';
   est_x_hist(i,:) = states';
   dist_hist(i,:)  = dist_estimation;
   hist_e(i,:)     = e;
   
end

 figure, set(gcf,'DefaultLineLineWidth',2.5);
 
 for i=1:6
 
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     plot(x_hist(:,i));
     plot(est_x_hist(:,i));
     legend('real','estimated')

     hold off
     
     grid on 
    
 end
 
 figure(); 
 plot(abs(hist_e))
 legend()
 figure(); 
 hold on 
 plot(dist_hist(:,:))
 hold off
 
function dx = nl_sys(states,u,constants)

    lf          = constants(1);
    lr          = constants(2);
    m           = constants(3);
    I           = constants(4);
    Cf          = constants(5);
    Cr          = constants(6);
    mu          = constants(7);

    vx          = states(1); 
    vy          = states(2); 
    w           = states(3);
    x           = states(4);
    y           = states(5);
    theta       = states(6);
    
    a           = u(2);
    delta       = u(1);
    
    a_F = atan((vy + lf*w)/vx) - delta;
    a_R = atan((vy - lr*w)/vx);

    FyF = -Cf * a_F;
    FyR = -Cr * a_R;
    
    F_drag = mu * vx;
%     F_drag  = 0;
    dx(1,1) = a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
    dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
    dx(4,1) = vx*cos(theta) - vy*sin(theta);
    dx(5,1) = vx*sin(theta) + vy*cos(theta);
    dx(6,1) = w;
end

 function E = update_E (alpha, vw, constants)

 %kia niro 

    lf          = constants(1);
    lr          = constants(2);
    m           = constants(3);
    I           = constants(4);
    Cf          = constants(5);
    Cr          = constants(6);
    mu          = constants(7);
    g           = constants(8); 
    
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

%     E = [ -mu, 0, 0, 0 ,0 ,0 ]';
    E = [ 0, -1/m, -(0.1)/I, 0 ,0 ,0 ]';

        
 end














