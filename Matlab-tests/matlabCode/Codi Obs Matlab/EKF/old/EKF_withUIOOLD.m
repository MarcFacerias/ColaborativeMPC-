% % with parameters
clear all

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
N           = 50;

constants = [lf, lr, m, I, Cf, Cr, mu, g, dt];

% % % noise symbolic variables
syms n1 n2 n3 n4 n5 n6

% Definitions of KF parame ters
Ew = [0.05   0.03   0.03  0.25  0.25  0.25];
Q = diag(Ew); % covariance matrix of the perturbations

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
u         = [0.25, 1.0]';
u_ant     = u;

% at each time instant we instantiate this model with x(k-1) and u(k-1) ->
% must be inside a for.

for i=2:N
    
   steer = u(1);
   accel = u(2);
   
   for j=1:10
        
       %simulation
       T = 0 : dt/5 : dt;
       [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ steer; accel ], constants), T, sim_states_ant);  

       x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)];
       
       %update variables
       vx      = states_ant(1); 
       vy      = states_ant(2); 
       w       = states_ant(3);
       x       = states_ant(4);
       y       = states_ant(5);
       theta   = states_ant(6);
       
       delta   = u_ant(1);
       a       = u_ant(2);

       % isntantiate partial derivatives
       A       = double(simplify(subs(A_sim)));
       B       = double(simplify(subs(B_sim)));
       
       % no es necesari, pero en alguns test feia la E variant
       E     = update_E(constants);
       
       % compute gamma and dy
       gamma   = pinv(C*E);
       dy    = (C*x_sim - y_meas)/dt;       
       
       %Ricatti steady state solution
       Ad = eye(6) + A*dt;

       %Definicio matrius + linearitzacio
       [K, S, P] = dlqr(Ad', C', Q, R);
       Azero = (I - E*gamma*C) * A ;
       Bzero = (I - E*gamma*C) * B ; 
       Azero_d = eye(6) + Azero*dt;
       Bzero_d = Bzero*dt;
       
       % estimacio de disturbance 
       dist_estimation = gamma*(dy - C * nl_sys(states_ant, u_ant, constants));
       
       % estimacio dels estats
       states = (Azero_d - K'*C)*states_ant + Bzero_d*u_ant - E*gamma*dy + K'*y_meas;
       
       %update variables
       e               = C * x_sim - C*states;
       states_ant      = states;
       sim_states_ant  = x_sim;
       y_meas          = C * x_sim;
       u_ant           = [steer;accel];

   end
   
   %guardar info 
   x_hist(i,:)     = x_sim';
   est_x_hist(i,:) = states';
   dist_hist(i,:)  = dist_estimation;
   hist_e(i,:)     = e;
end

 % plots
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
    % eq no lineals sense estimacio de frgament 
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
    
%     F_drag = mu * vx;
    F_drag  = 0;
    
    dx(1,1) = a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
    dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
    dx(4,1) = vx*cos(theta) - vy*sin(theta);
    dx(5,1) = vx*sin(theta) + vy*cos(theta);
    dx(6,1) = w;
end

 function E = update_E (constants)

 %matriu E

    lf          = constants(1);
    lr          = constants(2);
    m           = constants(3);
    I           = constants(4);
    Cf          = constants(5);
    Cr          = constants(6);
    mu          = constants(7);
    g           = constants(8); 
    
%     E = [ -mu         ,  0  ; ... 
%         
%           0       ,   -1/m ;                ...
%          
%           0 ,   -(lf-lr)/I ;                ...
%         
%           0         ,   0 ;                ...
%           0         ,   0 ;                ...
%           0         ,   0 ;                  ];

    E = [ -mu, 0, 0, 0 ,0 ,0 ]';
%     E = [ 0, -1/m, -(0.1)/I, 0 ,0 ,0 ]';

        
 end

function dx = nonlinear_model(t, states, u, constants)

    lf          = constants(1);
    lr          = constants(2);
    m           = constants(3);
    I           = constants(4);
    Cf          = constants(5);
    Cr          = constants(6);
    mu          = constants(7);

    % States
    vx      = states(1);
    vy      = states(2);
    w       = states(3);
    x       = states(4);
    y       = states(5);
    theta   = states(6);

    % Inputs:
    delta   = u(1);
    a       = u(2); 
    
    a_F = 0.0;
    a_R = 0.0;

    if abs(vx) > 0.1
        % Front and rear slip angles:
        a_F = atan((vy + lf*w)/vx) - delta;
        a_R = atan((vy - lr*w)/vx);
    end
    
    FyF = -Cf * a_F;
    FyR = -Cr * a_R;

    if abs(a_F) > 30.0/180.0*pi || abs(a_R) > 30.0/180.0*pi
        disp("WARNING: Large slip angles in simulation")
    end
    
     F_drag = mu * vx; %uncertainty
    
    % Equations of motion:
    dx(1,1) =   a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
    dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
    dx(4,1) = vx*cos(theta) - vy*sin(theta);
    dx(5,1) = vx*sin(theta) + vy*cos(theta);
    dx(6,1) = w;
    
end












