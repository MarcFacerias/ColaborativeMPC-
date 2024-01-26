clear all
clc

%% Loading data used for trainig
load data_for_planning.mat;
load curvature_for_planning;
load Iden_ANFIS_auton_car.mat;

% load Iden_ANFIS_Vx.mat;
% load Iden_ANFIS_Vy.mat;
% load Iden_ANFIS_Wz.mat; 

%% Trajectory Definition
Ts = 0.1;
map = Map("L_shape");
% map = Map("oval");

%% Track boundaries
[RightSide(1,:), RightSide(2,:), LeftSide(1,:), LeftSide(2,:)] = initializeFigure_xy( map, 1 );

%% MPC Initialization
Hp = 15;
Q  = diag([-10^-4  2.5  0.05  0.8  0.07]); % vx, vy, wz, y_e, theta_e
L  = [-0.35  0  0  0  0];
R  = diag([0.5, 0.09]); % delta, accel   

%% Initializing planner
s_ini(1) = 0;
for j = 2:Hp
    s_ini(j) = s_ini(j-1) + ( ( vehicle_states(j,1)*cos(vehicle_states(j,5)) - vehicle_states(j,2)*sin(vehicle_states(j,5)) ) / (1-vehicle_states(j,1)*curv(j)) ) * Ts;    
end

nx = length(Q);
nu = length(R);

% Creating Controller 
LPV_Controller = Planning_LPV_MPC_Fn_new(Hp, Q, R, L, Ts);

Xlast     = 0;
Ylast     = 0;
Thetalast = 0;
curvature = zeros(1,Hp);
vector_of_colors = {'g','m','c','r','k',};

SS_lap = 0;

% limits
left_limit  =  0.5*ones(1,Hp+1); 
right_limit = -0.5*ones(1,Hp+1);

% A_thetaE = zeros(Hp,6); OffS_thetaE = zeros(1,Hp);
% A_S      = zeros(Hp,5); OffS_S      = zeros(1,Hp);
% A_Ey     = zeros(Hp,3); OffS_Ey     = zeros(1,Hp);

% LPV parameters
A11 = zeros(1,1,Hp);   B11 = zeros(1,1,Hp);  
A12 = zeros(1,1,Hp);   B12 = zeros(1,1,Hp); 
A13 = zeros(1,1,Hp);   B21 = zeros(1,1,Hp); 
A22 = zeros(1,1,Hp);   B31 = zeros(1,1,Hp);
A23 = zeros(1,1,Hp);
A32 = zeros(1,1,Hp);
A33 = zeros(1,1,Hp);
A41 = zeros(1,1,Hp); 
A42 = zeros(1,1,Hp);
A51 = zeros(1,1,Hp); 
A52 = zeros(1,1,Hp);
A45 = zeros(1,1,Hp);

lf = 0.125;  lr = 0.125;  m  = 1.98;  
I  = 0.03;   Cf = 65;     Cr = 65;
g  = 9.81;   mu = 0.05;

fin_sim       = 200;
vx_acumm      = zeros(1,fin_sim);
vy_acumm      = zeros(1,fin_sim);
wz_acumm      = zeros(1,fin_sim);
steer_acumm   = zeros(1,fin_sim);
accel_acumm   = zeros(1,fin_sim);

%% Loop
for i = 1:fin_sim

   if i == 1
       
       for j = 1:Hp
           curvature(1,1,j) = Curvature(s_ini(j), map.PointAndTangent);
       end

       s_variation          = s_ini(Hp) - s_ini(Hp-1);
       curvature(1,1,Hp+1)  = Curvature(s_ini(Hp)+s_variation, map.PointAndTangent);
            
       for j = 1:Hp
           B11(1,1,j) = -Cf*sin(vehicle_inputs(j,1))/m;     
           B12(1,1,j) =  1;
           B21(1,1,j) =  Cf*cos(vehicle_inputs(j,1))/m; 
           B31(1,1,j) =  lf*Cf*cos(vehicle_inputs(j,1))/I;
           A11(1,1,j) = -mu;   
           A12(1,1,j) =  (sin(vehicle_inputs(j,1)) * Cf) / (m*vehicle_states(j,1));
           A13(1,1,j) =  (sin(vehicle_inputs(j,1)) * Cf * lf) / (m*vehicle_states(j,1)) + vehicle_states(j,2);
           A22(1,1,j) = -(Cr + Cf * cos(vehicle_inputs(j,1))) / (m*vehicle_states(j,1));
           A23(1,1,j) = -(lf * Cf * cos(vehicle_inputs(j,1)) - lr * Cr) / (m*vehicle_states(j,1)) - vehicle_states(j,1);
           A32(1,1,j) = -(lf * Cf * cos(vehicle_inputs(j,1)) - lr * Cr) / (I*vehicle_states(j,1));
           A33(1,1,j) = -(lf * lf * Cf * cos(vehicle_inputs(j,1)) + lr * lr * Cr) / (I*vehicle_states(j,1));
           A41(1,1,j) =  1/2*sin(vehicle_states(j,5));
           A45(1,1,j) =  1/2*vehicle_states(j,1);
           A42(1,1,j) =  cos(vehicle_states(j,5));
           A51(1,1,j) = -( curvature(1,1,j) * cos(vehicle_states(j,5))) / ( 1 - vehicle_states(j,4) * curvature(1,1,j) );
           A52(1,1,j) =  ( curvature(1,1,j) * sin(vehicle_states(j,5))) / ( 1 - vehicle_states(j,4) * curvature(1,1,j) );  
           
           [Ax(:,:,j), Bx(:,:,j), Cx(:,:,j)] = ANFIS_ABC_Comp(vehicle_states(j,1), vehicle_states(j,2), vehicle_states(j,3), vehicle_inputs(j,1), vehicle_inputs(j,2), Iden_ANFIS_Vx);
           [Ay(:,:,j), By(:,:,j), Cy(:,:,j)] = ANFIS_ABC_Comp(vehicle_states(j,1), vehicle_states(j,2), vehicle_states(j,3), vehicle_inputs(j,1), vehicle_inputs(j,2), Iden_ANFIS_Vy);
           [Aw(:,:,j), Bw(:,:,j), Cw(:,:,j)] = ANFIS_ABC_Comp(vehicle_states(j,1), vehicle_states(j,2), vehicle_states(j,3), vehicle_inputs(j,1), vehicle_inputs(j,2), Iden_ANFIS_Wz);
       end
       x0     = [1.0 0 0 0 0]'; % [vx, vy, w, y_e, theta_e]'
       past_u = [0 0]';
       s(i)   = 0;
       s_last = 0;
   else
       
       for j = 1:Hp
           curvature(1,1,j) = Curvature(s(j), map.PointAndTangent);
       end

       s_variation          = s(i+Hp-1) - s(i+Hp-2);
       curvature(1,1,Hp+1)  = Curvature(s(i+Hp-1) + s_variation, map.PointAndTangent);
            
       for j = 1:Hp  
           B11(1,1,j) = -Cf*sin(UU_opt(1,1,j))/m;     
           B12(1,1,j) =  1;
           B21(1,1,j) =  Cf*cos(UU_opt(1,1,j))/m; 
           B31(1,1,j) =  lf*Cf*cos(UU_opt(1,1,j))/I;
           A11(1,1,j) = -mu;   
           A12(1,1,j) =  (sin(UU_opt(1,1,j)) * Cf) / (m*XX_opt(1,1,j));
           A13(1,1,j) =  (sin(UU_opt(1,1,j)) * Cf * lf) / (m*XX_opt(1,1,j)) + XX_opt(2,1,j);
           A22(1,1,j) = -(Cr + Cf * cos(UU_opt(1,1,j))) / (m*XX_opt(1,1,j));
           A23(1,1,j) = -(lf * Cf * cos(UU_opt(1,1,j)) - lr * Cr) / (m*XX_opt(1,1,j)) - XX_opt(1,1,j);
           A32(1,1,j) = -(lf * Cf * cos(UU_opt(1,1,j)) - lr * Cr) / (I*XX_opt(1,1,j));
           A33(1,1,j) = -(lf * lf * Cf * cos(UU_opt(1,1,j)) + lr * lr * Cr) / (I*XX_opt(1,1,j));            
           A41(1,1,j) =  1/2*sin(XX_opt(5,1,j));
           A45(1,1,j) =  1/2*XX_opt(1,1,j);
           A42(1,1,j) =  cos(XX_opt(5,1,j));
           A51(1,1,j) = -( curvature(1,1,j) * cos(XX_opt(5,1,j))) / ( 1 - XX_opt(4,1,j) * curvature(1,1,j) );
           A52(1,1,j) =  ( curvature(1,1,j) * sin(XX_opt(5,1,j))) / ( 1 - XX_opt(4,1,j) * curvature(1,1,j) ); 
           [Ax(:,:,j), Bx(:,:,j), Cx(:,:,j)] = ANFIS_ABC_Comp(XX_opt(1,1,j), XX_opt(2,1,j), XX_opt(3,1,j), UU_opt(1,1,j), UU_opt(2,1,j), Iden_ANFIS_Vx);
           [Ay(:,:,j), By(:,:,j), Cy(:,:,j)] = ANFIS_ABC_Comp(XX_opt(1,1,j), XX_opt(2,1,j), XX_opt(3,1,j), UU_opt(1,1,j), UU_opt(2,1,j), Iden_ANFIS_Vy);
           [Aw(:,:,j), Bw(:,:,j), Cw(:,:,j)] = ANFIS_ABC_Comp(XX_opt(1,1,j), XX_opt(2,1,j), XX_opt(3,1,j), UU_opt(1,1,j), UU_opt(2,1,j), Iden_ANFIS_Wz); 
       end
       % x0     = XX_opt(:,1,1);
       % PASTU  = UU_opt(:,1,1);
       past_u = UU_opt(:,1,1);
   end

%     % Obstacles L-shape
%     Obs_Pose1        = [2,2];          % [x,y]
%     Obs_Pose2        = [-0.5,-0.05];   % [x,y]
    
    % Obstacles oval
    Obs_Pose1        = [0, 2.7];    % [x,y]
    Obs_Pose2        = [1, 3];    	% [x,y]

    [left_limit, right_limit] = Obs_Avoidance_2vehicles(Obs_Pose1, Obs_Pose2, map, s, i, Hp, SS_lap);

   % Solving optimization problem
   inputs_MPC = {x0, past_u, B11, B12, B21, B31, A11, A12, A13, A22,...
          A23, A32, A33, A41, A42, A51, A52, A45, Ax, Ay, Aw, Bx, By, Bw, Cx, Cy, Cw, right_limit, left_limit};
   [solutions, diagnostics, objective] = LPV_Controller{inputs_MPC};
    
   if diagnostics == -1
       UU_optim = [0;0];
   elseif diagnostics == 0
       UU_opt = solutions{1};
       XX_opt = solutions{2};
   else
       error('The problem is infeasible')
   end
    
   % Middle-road computation
   Xref(1)     = Xlast;
   Yref(1)     = Ylast;
   Thetaref(1) = Thetalast; 
   for j = 1:length(XX_opt(1,:))
       s(i+j) = s(i+j-1) + ( ( XX_opt(1,1,j)*cos(XX_opt(5,1,j)) - XX_opt(2,1,j)*sin(XX_opt(5,1,j)) ) / (1-XX_opt(4,1,j)*curvature(1,1,j)) ) * Ts;
       [Xref(j+1), Yref(j+1), Thetaref(j+1)] = map.getGlobalPosition( s(i+j), 0 ); 
   end
   Xlast     = Xref(2);
   Ylast     = Yref(2);
   Thetalast = Thetaref(2);
    
   % Error-based pose computation
   for j = 1:length(XX_opt(1,:))
       theta(j)  = wrap(Thetaref(j) - XX_opt(5,1,j));      % yaw
       xp(i,j)   = Xref(j) - XX_opt(4,1,j)*sin(theta(j));  % position_x
       yp(i,j)   = Yref(j) + XX_opt(4,1,j)*cos(theta(j));  % position_y
   end

%    for j = 1:size(XX_opt,3)
%        if j==1
%            % Middle-road computation
%            s(j) = s_last + ( ( XX_opt(1,1,j)*cos(XX_opt(5,1,j)) - XX_opt(2,1,j)*sin(XX_opt(5,1,j)) ) / (1-XX_opt(4,1,j)*curvature(1,1,j)) ) * Ts;
%            [Xref(j), Yref(j), Thetaref(j)] = map.getGlobalPosition( s(j), 0 ); 
%            
%            % Error based computation
%            theta(j)  = wrap(Thetaref(j) - XX_opt(5,1,j));   % Yaw
%            xp(j)   = Xref(j) - XX_opt(4,1,j)*sin(theta(j)); % X_position
%            yp(j)   = Yref(j) + XX_opt(4,1,j)*cos(theta(j)); % Y_position
%        else
%            s(j) = s(j-1) + ( ( XX_opt(1,1,j)*cos(XX_opt(5,1,j)) - XX_opt(2,1,j)*sin(XX_opt(5,1,j)) ) / (1-XX_opt(4,1,j)*curvature(1,1,j)) ) * Ts;
%            [Xref(j), Yref(j), Thetaref(j)] = map.getGlobalPosition( s(j), 0 ); 
%            
%            theta(j) = wrap(Thetaref(j) - XX_opt(5,1,j));     % Yaw
%            xp(j)    = Xref(j) - XX_opt(4,1,j)*sin(theta(j)); % X_position
%            yp(j)    = Yref(j) + XX_opt(4,1,j)*cos(theta(j)); % Y_position
%        end
%    end
   
    % Obstacle avoidance corridor plot
    for j = 1:length(XX_opt(1,:)) % left
        xp_L_Obs(i,j) = Xref(j) - left_limit(j)*sin(Thetaref(j));
        yp_L_Obs(i,j) = Yref(j) + left_limit(j)*cos(Thetaref(j));
    end
    
    for j = 1:length(XX_opt(1,:)) % right
        xp_R_Obs(i,j) = Xref(j) - right_limit(j)*sin(Thetaref(j));
        yp_R_Obs(i,j) = Yref(j) + right_limit(j)*cos(Thetaref(j));
    end

   s_last = s(1);
   
   s_acumm(i,1)     = s(1);
   theta_acumm(i,1) = theta(1);
   X_acumm(i,1)     = xp(1);
   Y_acumm(i,1)     = yp(1);

   % Vehicle simulation
   T = Ts*i:Ts/(Ts*1000):Ts*i+Ts; % integration time in ODE45
   INPUTS(i,:) = [UU_opt(1,1,1) UU_opt(2,1,1)];
   
   % Non-linear simulation vehicle
   [T,x] = ode45(@(t,x) cartesian_nonlinear_Berkeley_model...
        (t,x,[UU_opt(1,1,1); UU_opt(2,1,1); s(1)]),T,x0);
    
    % Plotting
    x_sim = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5)];
    vx_accum(i,1)      = x_sim(1);
    vy_accum(i,1)      = x_sim(2);
    wz_accum(i,1)      = x_sim(3);
    y_e_accum(i,1)     = x_sim(4);
    theta_e_accum(i,1) = x_sim(5);

%     vx_accum(i,1)      = XX_opt(1,1,1);
%     vy_accum(i,1)      = XX_opt(2,1,1);
%     wz_accum(i,1)      = XX_opt(3,1,1);
%     y_e_accum(i,1)     = XX_opt(4,1,1);
%     theta_e_accum(i,1) = XX_opt(5,1,1);

    steer_accum(i,1)   = UU_opt(1,1,1);
    accel_accum(i,1)   = UU_opt(2,1,1);
    
    % Vx
    clf(figure(2))
    figure(2)
    n = 1;
    subplot(n,1,1)
    plot(vx_accum,'r','LineWidth',3)
    ylabel('V_x [m/s]'), xlabel('k')
    grid on
    
    % Vy
    figure(3)
    clf(figure(3))
    n = 1;
    subplot(n,1,1)
    plot(vy_accum,'r','LineWidth',3)
    ylabel('V_y [m/s]'), xlabel('k')
    grid on
    
    % Wz
    clf(figure(4))
    figure(4)
    n = 1;
    subplot(n,1,1)
    plot(wz_accum,'r','LineWidth',3)
    ylabel('W_z [rad/s]'), xlabel('k')
    grid on
    
    % Control inputs
    figure(5),
    clf(figure(5)) 
    subplot(2,1,1)
    plot(steer_accum, 'r','LineWidth',3)
    ylabel('Steering angle'), grid on
    subplot(2,1,2)
    plot(accel_accum, 'k','LineWidth',3)
    ylabel('Acceleration'), grid on
    
    % Planned trajectory
    clf(figure(6))
    figure(6)
    hold on
    plot(RightSide(1,:),RightSide(2,:),'k', LeftSide(1,:),LeftSide(2,:),'k'),grid on
    plot(Xref(1:end-1),Yref(1:end-1),'--k')
    hold on, plot(xp(i,1), yp(i,1),'gs', 'LineWidth',2,'MarkerSize',10,...
    'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5])
    hold on, plot(xp(i,1:end-1), yp(i,1:end-1), 'r','LineWidth',3);
    hold on, plot(Obs_Pose1(1), Obs_Pose1(2),'gs', 'LineWidth',2,...
    'MarkerSize',10, 'MarkerEdgeColor','k','MarkerFaceColor',[0.2,0.8,0.5])
    hold on, plot(Obs_Pose2(1), Obs_Pose2(2),'gs', 'LineWidth',2,...
    'MarkerSize',10, 'MarkerEdgeColor','k','MarkerFaceColor',[0.2,0.8,0.5])
    hold on, plot(xp_R_Obs(i,:), yp_R_Obs(i,:), 'k','LineWidth',3);
    hold on, plot(xp_L_Obs(i,:), yp_L_Obs(i,:), 'k','LineWidth',3);
    drawnow
end

vec = [10, 10, 900, 550];
line_border = 1.5;
font_size = 20;

figure('position', vec, 'Color', [1 1 1]);
[RightSide2(1,:), RightSide2(2,:), LeftSide2(1,:), LeftSide2(2,:)] = initializeFigure_xy(map, 1);
hold on
plot(RightSide(1,:),RightSide(2,:),'k', LeftSide(1,:), LeftSide(2,:),'k')
plot(RightSide2(1,:),RightSide2(2,:),'--r', 'linewidth', 3)
plot(LeftSide2(1,:),LeftSide2(2,:),'--r', 'linewidth', 3)
vx = vx_accum;
hold on
tr28 = scatter(X_acumm, Y_acumm, 25, vx, 'filled');
c = colorbar;%('AxisLocation','in');
% c.Location = 'northoutside';
% c.TickLabelInterpreter = 'latex';
c.Label.Interpreter = 'latex';
c.Label.FontSize = font_size;
c.Label.String = '$v_x$ $(m/s)$';
colormap(turbo); 
set(gca,'FontSize',font_size);
xlabel('$X (m)$','interpreter','latex','fontsize',font_size)
ylabel('$Y (m)$','interpreter','latex','fontsize',font_size)
% legend([tr27 tr28],'Reference','Response with $SoC$ objective','fontsize',16,'Orientation','horizontal','Location','northeast','interpreter','latex');
