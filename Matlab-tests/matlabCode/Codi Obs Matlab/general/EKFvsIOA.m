% Test Observer
clear all 
% data  =[states(6),acc,steer]
% initialization
% addpath '/home/marc/IRI_Internship/tfm/matlabCode/Codi Obs Matlab'
MAIN_Observer(); %computar obs

%%


dt = 0.005;

x_ant = zeros(6,1);
x_ant(1) = 1.0112;
x_ant(2) = 0.0115;
x_ant(3) = 0.0946;
states_ant = x_ant;

% x_ant = STATES(:,2);
 
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
               
Ew    = diag([ 0.05   0.03   0.03  0.25  0.25  0.25 ]);  	% [ vx, vy, wz, x, y, theta ]
Ev    = diag([ 0.01   0.02   0.02  0.01  0.01 ]);              % measured variables
u     = [0.1,2.0]';
u_ant = u;
%%
%%INI EKF

syms vx vy w x y theta a delta 

lf          = 0.125;
lr          = 0.125;
m           = 1.98;
I           = 0.03;
Cf          = 60;
Cr          = 60;  
mu          = 0.1;
g           = 9.8;
dt          = 0.005;
N           = 200;

constants = [lf, lr, m, I, Cf, Cr, mu, g, dt];

% % % without parameters
% % syms vx vy psiDot x y psi a delta lf lr m I Cf Cr mu 

% % % noise symbolic variables
syms n1 n2 n3 n4 n5 n6
% syms vx vy w x y theta a delta lf lr m I Cf Cr mu 

% Definitions of KF parame ters
% Ew = [0.1,0.2,0.3,0.4,0.5,.6]; % covariance of the perturbations
Ew_vec = [0.05   0.03   0.03  0.25  0.25  0.25];
Q = diag(Ew_vec); % covariance matrix of the perturbations

% Ev = [0.1,0.2,0.3,0.4,0.5]; %covariance of the measurement noise
Ev_vec = [0.01   0.02   0.02  0.01  0.01];
R = diag(Ev_vec); % covariance matrix of the measurement noise

% Eq of the sys
a_F = atan((vy + lf*w)/vx) - delta;
a_R = atan((vy - lr*w)/vx);

FyF = -Cf * a_F;
FyR = -Cr * a_R;


dx1 = a - mu*vx - FyF*sin(delta)/m  +  w*vy + n1*Ew(1); 
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


%%

noiseSignal = (rand(1, N)-0.5)*5;
        
for i=2:N
    
  
   steer = u(1);
   accel = u(2);
   
   for j=1:10

       T = dt*j : dt/5 : dt*j+dt;

       [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ steer; accel ]), T, x_ant);  

       x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)];

       y_out = C * x_sim + noiseSignal(i) * [0.01   0.02   0.02  0.01  0.01]';

       %IOA
       vx = cxio(1);
       vy = cxio(2);
       theta = cxio(6);
       steer = u(1);

       [ L ] = K_Computation( steer, vx, vy, theta, SchedVars_Limits, Llmi ); 

       [ A, B, ~, ~, ~, ~ ] = Continuous_AB_Comp( steer, ...
           vx, vy, theta, C, 'Varianttt B' );

        A_obs = eye(6) + dt*A;
        B_obs = dt*B;
        L_obs = -dt*L;

       % center    
       xeio=A_obs*cxio + B_obs*u;
       cxxio = xeio + (L_obs*(y_ant-(C*xeio))); % 2.7a

       %eixos   

       shape_priori = [A_obs*Rxio, Ew];
       Rxxio = [(eye(6) - L_obs*C)*shape_priori,- L_obs*Ev];
       
%        EKF
       
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

%        Ricatti steady state solution
       Ad = eye(6) + A*dt;
       Bd = B*dt;
       [K, S, P] = dlqr(Ad', C', Q, R);

       x_pred  = states_ant + nl_sys(states_ant, u_ant, constants) * dt;

       e = y_ant - C*x_pred;
       states = x_pred + K'*e;

       states_ant      = states;
       u_ant           = [steer;accel];
       
       cxio = cxxio;
       Rxio = Rxxio;
       x_ant = x_sim;
       y_ant = y_out;
       
   end
   
   for j=1:6
       
       Rxio_red = reduction(Rxxio,n_red);
       Exio     = envbox(Rxio);
       xmax(i,j)    = cxio(j) + abs(Exio (j,j));
       xmin(i,j)    = cxio(j) - abs(Exio (j,j));
       center(i,j)  = cxio(j);
       
   end
  
    x_hist(i,:) = (x_sim + noiseSignal(i) * [ 0.01 0 0.02 0.02 0.01 0.01 ]')';
    est_x_hist(i,:) = states';
    i
    
end

 figure, set(gcf,'DefaultLineLineWidth',2.5);
 for i=1:6
 
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     % IOA
      plot(xmax(:,i));
      plot(center(:,i),'r');
      plot(x_hist(:,i),'b');
      plot(xmin(:,i));
      plot(est_x_hist(:,i));

     hold off
     legend('max','center','sim states','min','EKF states')
     grid on 
    
 end
 
  figure, set(gcf,'DefaultLineLineWidth',2.5);
 for i=1:6
 
     subplot(6,1,i)
     hold on
     ylabel(strcat('x',num2str(i)))
     
     % IOA
      plot(est_x_hist(:,i));
      plot(center(:,i),'r');
      plot(x_hist(:,i),'b');

     hold off
     legend('center','sim states','EKF states')
     grid on 
    
 end
 
 function dx = nl_sys(states,u,constants)

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
    delta       = u(1);
    
    a_F = atan((vy + lf*w)/vx) - delta;
    a_R = atan((vy - lr*w)/vx);

    FyF = -Cf * a_F;
    FyR = -Cr * a_R;
    
    F_drag = mu * vx;

    dx(1,1) = a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
    dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
    dx(4,1) = vx*cos(theta) - vy*sin(theta);
    dx(5,1) = vx*sin(theta) + vy*cos(theta);
    dx(6,1) = w;
end