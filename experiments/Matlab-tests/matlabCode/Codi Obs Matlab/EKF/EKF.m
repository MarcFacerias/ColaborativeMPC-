clear all
path = genpath('/home/marc/IRI_Internship/tfm/matlabCode/Codi Obs Matlab');
addpath(path);
% simbolic constant definition to instantiate Jacobain
syms vx vy w x y theta a delta 

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
N           = 250;

constants = [lf, lr, m, I, Cf, Cr, mu, g, dt];

% Definitions of KF parameters
Ew = [0.05   0.03   0.03  0.25  0.25  0.25];
Q = (diag(Ew)); % covariance matrix of the perturbations

Ev = [0.01   0.02   0.02  0.01  0.01];
R = (diag(Ev)); % covariance matrix of the measurement noise

P = zeros(6,6);

%OLD MODEL -> USED BY ME
% Eq of the sys
% a_F = atan((vy + lf*w)/vx) - delta;
% a_R = atan((vy - lr*w)/vx);
% 
% FyF = -Cf * a_F;
% FyR = -Cr * a_R;
% 
% F_drag = mu * vx;
% 
% dx1 = a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
% dx2 = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
% dx3 = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
% dx4 = vx*cos(theta) - vy*sin(theta);
% dx5 = vx*sin(theta) + vy*cos(theta);
% dx6 = w;

%NEW MODEL -> USED BY EUGE
dx1 = a + w*(vy + (Cf*lf*sin(delta))/(m*vx)) - (Cf*delta*sin(delta))/m + (Cf*vy*sin(delta))/(m*vx);
dx2 = (Cf*delta*cos(delta))/m - (vy*(Cr + Cf*cos(delta)))/(m*vx) - w*(vx - (Cr*lr - Cf*lf*cos(delta))/(m*vx));
dx3 = (vy*(Cr*lr - Cf*lf*cos(delta)))/(I*vx) - (w*(Cf*cos(delta)*lf^2 + Cr*lr^2))/(I*vx) + (Cf*delta*lf*cos(delta))/I;
dx4 = vx*cos(theta) - vy*sin(theta);
dx5 = vy*cos(theta) + vx*sin(theta);
dx6 = w; 

%Landmark Eq
dx7 = x_lm;
dx8 = y_lm;

% evaluate in x where we are doing the linearization 
tic
Jb = simplify(jacobian([dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8]',[vx; vy; w; x; y; theta; x_lm; y_lm delta; a]));
toc

% we obtain a 6x8 matrix -> first 6 columns A, 2 last columns B  this leads
% to f(x) = f(x0) + A(x0)*(x-x0) + B(u0)*(u-u0)

A_sim = Jb(:,1:6);
B_sim = Jb(:,7:8);

C_robot = [ 1 0 0 0 0 0; %equivalent to Hx
          0 0 1 0 0 0;
          0 0 0 1 0 0;
          0 0 0 0 1 0;
          0 0 0 0 0 1];    
      
% initial conditions that avoid singularity at vx = 0
states_ant = zeros(6,1);
states_ant(1) = 1.0112;
states_ant(2) = 0.0115;
states_ant(3) = 0.0946;
sim_states_ant = states_ant;

y_meas    = zeros(5,1);
y_meas(1) = 1.0112;
y_meas(3) = 0.0946;
u_ant     = [0, 0]';
u         = [0.25, 1.0]';
dx_ant    = 0;


% at each time instant we instantiate this model with x(k-1) and u(k-1) 

% offline noise definition
a = -1.0;
b = 1.0;
noiseSignal = a + (b-a).*rand(1,N);

% Simulation loop 
for i=2:N
    
   % Update control actions -> control loop
   steer = u(1);
   accel = u(2);
   
   for j=1:10
       % Execute estimation loop 10 times for each control loop to ensure
       % steady state of the observer
       
       if i > 30 && i < 40
           dist = 0;
       else
           dist = 0;
       end
       
       tic 
       T = 0 : dt/5 : dt;
       % solve the diferential eq taking as initial point the previous
       % simulated state
       
       [ T, x ] = ode45(@( t,x ) nonlinear_model( t, x, [ steer; accel ], constants, Ew), T, sim_states_ant);  
%        [ T, x ] = ode45(@( t,x ) nonlinear_model_disturbed( t, x, [ u_ant(1); u_ant(2) ], dist, constants), T, sim_states_ant);  

       x_sim    = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)];
       
       % Parse the states and update Jacobians
       
       vx      = states_ant(1); 
       vy      = states_ant(2); 
       w       = states_ant(3);
       x       = states_ant(4);
       y       = states_ant(5);
       theta   = states_ant(6);
       
       delta   = u_ant(1);
       a       = u_ant(2);
       
       % instantiate partial derivatives
       A       = double(simplify(subs(A_sim)));
       B       = double(simplify(subs(B_sim)));

       %discrete Ricatti steady state solution
       Ad = eye(6) + A*dt;
       Bd = B*dt;

        % Eq for the recursive version of the Kalman filter (not used as we change the model at each ts)
       % %  correction step + prediction of P: (not solving steady state ricati eq)
         P_pred  = Ad*P*Ad' + Q;
         x_pred  = states_ant + nl_sys(states_ant, u_ant, constants) * dt; % prediction step using non-linear model 

         e = y_meas - C*x_pred; % z in the implementation guide
         Z = C*P_pred*C' + R;
         K = P_pred*C'*inv(Z);
         P = P_pred - K*C*P_pred;
         
         states = x_pred + K*e;
       

       %update constants for next iteration
       states_ant      = states;
       sim_states_ant  = x_sim;
       y_meas          = C * x_sim;
       u_ant           = [steer;accel];

       toc
   end
   
   %Saving historic of values
   x_hist(i,:)     = x_sim';
   est_x_hist(i,:) = states';
   hist_e(i,:)       = e;
   states
   x_sim
end

 % Plot states vs "real states" (taking into account the noise)
 figure, set(gcf,'DefaultLineLineWidth',2.5);
 sgtitle('EKF Estimator')
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
 
 % Plot absolute error
 figure(); 
 plot(abs(hist_e))
   
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
    delta       = u(1);
    
    a_F = atan((vy + lf*w)/vx) - delta;
    a_R = atan((vy - lr*w)/vx);

    FyF = -Cf * a_F;
    FyR = -Cr * a_R;
    
%     F_drag = mu * vx;
    F_drag = 0;

    dx(1,1) = a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
    dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I  ; 
    dx(4,1) = vx*cos(theta) - vy*sin(theta);
    dx(5,1) = vx*sin(theta) + vy*cos(theta);
    dx(6,1) = w;
end


  


function dx = nonlinear_model(t, states, u, constants, Ew)

    lf          = constants(1);
    lr          = constants(2);
    m           = constants(3);
    I           = constants(4);
    Cf          = constants(5);
    Cr          = constants(6);
    mu          = constants(7);
    
    a = -1.0;
    b = 1.0;
    noiseSignal = a + (b-a).*rand(1,6);

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
    dx(1,1) = a - F_drag  -  FyF*sin(delta)/m  +  w*vy + Ew(1)*noiseSignal(1);    
    dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx + Ew(2)*noiseSignal(2) ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I + Ew(3)*noiseSignal(3) ; 
    dx(4,1) = vx*cos(theta) - vy*sin(theta) + Ew(4)*noiseSignal(4);
    dx(5,1) = vx*sin(theta) + vy*cos(theta) + Ew(5)*noiseSignal(5);
    dx(6,1) = w + Ew(6)*noiseSignal(6);
       
end












