%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 18/12/2018
% UPC
% Planner usando LPV-MPC.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

yalmip('clear')
clear
clc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LPV-MPC DESIGN: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


load('NL_vars.mat');


% map = Map("L_shape");
% map = Map("3110");
map = Map("oval");

[RightSide(1,:), RightSide(2,:), LeftSide(1,:), LeftSide(2,:)] = initializeFigure_xy( map, 1 );
% [RightPB(1,:), RightPB(2,:), LeftPB(1,:), LeftPB(2,:)] = initializeFigure_xy( map, 0 );

% Hp = length(ss_NL);
Hp = 15; % Enough for the oval and 3110 tracks, but not for L-shape track
% Hp = 20; % May be enough for L-shape track

Tss = 0.1;

lf = 0.125;
lr = 0.125;
m  = 1.98;
I  = 0.03;
Cf = 70;
Cr = 70;

LPV_MPC_controller = LPV_MPC_fnc_dt_Vnew( Hp, Tss );


tXlast = 0;
tYlast = 0;
tThetalast = 0;
curvature_dt = zeros(1,Hp+1);
vector_of_colors = {'g','m','c','r','k',};


SS_lap = 0;
left_limit = 0.5*ones(1,Hp+1); right_limit = -0.5*ones(1,Hp+1);


% Vector memory allocation:
A_1 = zeros(1,Hp);  A_3 = zeros(1,Hp); A_5 = zeros(1,Hp);
A_2 = zeros(1,Hp);  A_4 = zeros(1,Hp); A_6 = zeros(1,Hp);
A_7 = zeros(1,Hp);  A_8 = zeros(1,Hp); A_9 = zeros(1,Hp);
A_10 = zeros(1,Hp); B_1 = zeros(1,Hp); B_2 = zeros(1,Hp);
B_3 = zeros(1,Hp);  B_4 = zeros(1,Hp);
A_thetaE = zeros(Hp,6); OffS_thetaE = zeros(1,Hp);
A_S = zeros(Hp,5); OffS_S = zeros(1,Hp);
A_Ey = zeros(Hp,3); OffS_Ey = zeros(1,Hp);



TOT_SIM     = 130;
timeElapsed = zeros(1,TOT_SIM);
Vx_Tot      = zeros(1,TOT_SIM);
Vy_Tot      = zeros(1,TOT_SIM);
W_Tot       = zeros(1,TOT_SIM);
Accel_Tot   = zeros(1,TOT_SIM);
Steer_Tot   = zeros(1,TOT_SIM);
LongA_Tot   = zeros(1,TOT_SIM);
LatA_Tot    = zeros(1,TOT_SIM);




for k=1:TOT_SIM
    k
      
    if k == 1
        
        for j = 1:Hp
            curvature_dt(j) = Curvature(ss_NL(j), map.PointAndTangent);
        end
        s_variation = ss_NL(Hp) - ss_NL(Hp-1);
        curvature_dt(Hp+1) = Curvature(ss_NL(Hp)+s_variation, map.PointAndTangent);
        
        for j=1:Hp 
            
            A_1(1,j) = 1 / ( 1-ey_NL(j)*curvature_dt(j) );
            A_2(1,j) = sin(etheta_NL(j));   
            A_3(1,j) = Vy_NL(j); 
            A_4(1,j) = Vx_NL(j); 
            A_5(1,j) = (sin(delta(j)) * Cf) / (m*Vx_NL(j));            
            A_6(1,j) = (sin(delta(j)) * Cf * lf)/(m*Vx_NL(j)) + Vy_NL(j);            
            
            A_7(1,j) = -(Cr + Cf * cos(delta(j))) / (m*Vx_NL(j));
            A_8(1,j) = -(lf * Cf * cos(delta(j)) - lr * Cr) / (m*Vx_NL(j))  - Vx_NL(j);
            A_9(1,j) = -(lf * Cf * cos(delta(j)) - lr * Cr) / (I*Vx_NL(j));
            A_10(1,j)= -(lf * lf * Cf * cos(delta(j)) + lr * lr * Cr) / (I*Vx_NL(j));
            
            %             AA(:,:,j) =    [0,      A_5(1,j)        , A_6(1,j)    , 0    , 0    , 0;
            %                             0     , A_7(1,j)        , A_8(1,j)    , 0    , 0    , 0;
            %                             0     , A_9(1,j)        , A_10(1,j)   , 0    , 0    , 0;
            %                             0     , 1        , 0    , 0    , A_4(1,j)    , 0;
            %                         -A_1(1,j)*curvature_dt(j), A_1(1,j)*A_2(1,j)*curvature_dt(j) , 1   , 0    , 0    , 0;
            %                             0     ,        0 , 0    , 0    , 0    , 0]; 
            %                         
            %             AA(:,:,j) = eye(6) + 0.1*AA(:,:,j);
            
            B_1(1,j) = -(sin(delta(j)) * Cf) / m;
            B_2(1,j) = 1;
            B_3(1,j) = (Cf * cos(delta(j))) / m; 
            B_4(1,j) = (lf * Cf * cos(delta(j))) / I;
            
        end
        x0 = [0.97 0 0 0 0 ]';       % [vx, vy, w, ey, etheta]
        PASTU = [0 0]';
        s(k) = 0;
                
    else

        for j = 1:Hp
            curvature_dt(j) = Curvature(s(k+j-1), map.PointAndTangent);
        end
        s_variation = s(k+Hp-1) - s(k+Hp-2);
        curvature_dt(Hp+1) = Curvature(s(k+Hp-1)+s_variation, map.PointAndTangent);
        
        for j=1:Hp
            
            A_1(1,j) = 1 / ( 1-XX_dt(4,j)*curvature_dt(j) );
            A_2(1,j) = sin(XX_dt(5,j));            
            A_3(1,j) = XX_dt(2,j);
            A_4(1,j) = XX_dt(1,j);
            A_5(1,j) = (sin(UU_dt(1,j)) * Cf) / (m*XX_dt(1,j));
            A_6(1,j) = (sin(UU_dt(1,j)) * Cf * lf)/(m*XX_dt(1,j)) + XX_dt(2,j);            
            A_7(1,j) = -(Cr + Cf * cos(UU_dt(1,j))) / (m*XX_dt(1,j));
            A_8(1,j) = -(lf * Cf * cos(UU_dt(1,j)) - lr * Cr) / (m*XX_dt(1,j))  - XX_dt(1,j);
            A_9(1,j) = -(lf * Cf * cos(UU_dt(1,j)) - lr * Cr) / (I*XX_dt(1,j));
            A_10(1,j)= -(lf * lf * Cf * cos(UU_dt(1,j)) + lr * lr * Cr) / (I*XX_dt(1,j));
            
            B_1(1,j) = -(sin(UU_dt(1,j)) * Cf) / m;
            B_2(1,j) = 1;
            B_3(1,j) = (Cf * cos(UU_dt(1,j))) / m; 
            B_4(1,j) = (lf * Cf * cos(UU_dt(1,j))) / I;     
            
        end        
        x0      = XX_dt(:,1);
        PASTU   = UU_dt(:,1);
    end
    
    
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Obstacle avoidance code:     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     if k > 0 && k < 200
    %         Obs_Pose        = [1.85,1.9];    	%[x,y]
    %         [left_limit, right_limit] = Obs_Avoidance( Obs_Pose, map, s, k, Hp, SS_lap );
    %     else
    %         Obs_Pose        = [-2,1.5];       %[x,y]
    %     end

    % Obstaculos L-shape:
    Obs_Pose1        = [2,2];          %[x,y]
%     Obs_Pose1        = [1, 0.05];          %[x,y]
    Obs_Pose2        = [-0.5,-0.05];    	%[x,y]
    
    % Obstaculos oval:
    Obs_Pose1        = [0, 2.7];          %[x,y]
    Obs_Pose2        = [1, 3];    	%[x,y]

%     Obs_Pose1        = [1, 2.7];    %[x,y]
%     Obs_Pose2        = [0, 3];    	%[x,y]
    
    [left_limit, right_limit] = Obs_Avoidance_2vehicles( Obs_Pose1, Obs_Pose2, map, s, k, Hp, SS_lap );
    
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Planning code:     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    inputs = {x0, curvature_dt, PASTU, A_1, A_2, A_3, A_4, A_5, A_6, A_7, A_8,...
        A_9, A_10, B_1, B_2, B_3, B_4, right_limit, left_limit};
    
    tic
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [solutions,diagnostics] = LPV_MPC_controller{inputs};
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    timeElapsed(k) = toc;
    
    if diagnostics == -1
        Uopt = [0;0];
    elseif diagnostics == 0
        UU_dt   = solutions{1}; 
        XX_dt   = solutions{2};
    else         
        error('Nan result.')
    end
    
    % Middle-road computation:
    tXref(1)    = tXlast;
    tYref(1)    = tYlast;
    tThetaref(1)= tThetalast;
    for j = 1:length(XX_dt(1,:))
        s(k+j) = s(k+j-1) + ( ( XX_dt(1,j)*cos(XX_dt(5,j)) - XX_dt(2,j)*sin(XX_dt(5,j)) ) / (1-XX_dt(4,j)*curvature_dt(j)) ) * Tss;
        [tXref(j+1), tYref(j+1), tThetaref(j+1)] = map.getGlobalPosition( s(k+j), 0 );
    end
    tXlast = tXref(2);
    tYlast = tYref(2);
    tThetalast = tThetaref(2);
    
    %%% Checking zero cross:
    cross = zero_cross( tThetaref, tXref, tYref); % s(end-Hp:end) );
    
    if cross == 1
        SS_lap = s(k);
    end
    
    % Error-based pose computation:
    for i=1:length(XX_dt(1,:))
        yaw(i)  = tThetaref(i) + XX_dt(5,i);
        xp(k,i) = tXref(i) - XX_dt(4,i)*sin(yaw(i));
        yp(k,i) = tYref(i) + XX_dt(4,i)*cos(yaw(i));
    end
    
    for i=1:length(XX_dt(1,:))-1
        long_acc(i) = ((XX_dt(1,i+1)-XX_dt(1,i))/0.1);
        lat_acc(i)  = ((XX_dt(2,i+1)-XX_dt(2,i))/0.1);
    end

    
    
    %%%% Obstacle avoidance corridor plot:
    for i=1:length(XX_dt(1,:))
        xp_L_Obs(k,i) = tXref(i) - left_limit(i)*sin(tThetaref(i));
        yp_L_Obs(k,i) = tYref(i) + left_limit(i)*cos(tThetaref(i));
    end
    for i=1:length(XX_dt(1,:))
        xp_R_Obs(k,i) = tXref(i) - right_limit(i)*sin(tThetaref(i));
        yp_R_Obs(k,i) = tYref(i) + right_limit(i)*cos(tThetaref(i));
    end
    %     plot(xp_R_Obs(k,:), yp_R_Obs(k,:), 'k','LineWidth',3);
    %     hold on, plot(xp_L_Obs(k,:), yp_L_Obs(k,:), 'k','LineWidth',3);
    
    
    
    % Almacenamos variables para plotearlas:
    Vx_Tot(1,k)     = XX_dt(1,1);
    Vy_Tot(1,k)     = XX_dt(2,1);
    W_Tot(1,k)      = XX_dt(3,1);
    Accel_Tot(1,k)  = UU_dt(2,1);
    Steer_Tot(1,k)  = UU_dt(1,1);
    LongA_Tot(1,k)  = long_acc(1);
    LatA_Tot(1,k)   = lat_acc(1);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%   PLOTTING SECTION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ACCELERATIONS FIGURE:
    figure(1),
    clf(figure(1)) 
    subplot(2,1,1), plot(LongA_Tot, 'r','LineWidth',3), ylabel('X_{acc}'), grid on
    subplot(2,1,2), hold on, plot(LatA_Tot, 'k','LineWidth',3), ylabel('Y_{acc}')
    grid on
    
%     clf(figure(4)),figure(4)
%     nn = 2;
%     for i = 1:length(long_acc)-1
%         circle(0,0,1)
%         hold on, plot([0 0],[1 -1],'--k')
%         hold on, plot([1 -1],[0 0],'--k')
% 
%         if i==1
%             hold on, plot(lat_acc(i), long_acc(i), '-o','MarkerSize',20,...
%             'MarkerEdgeColor','blue','MarkerFaceColor',[0 0 0])
%         else
%             hold on, plot(lat_acc(i), long_acc(i), '-x','MarkerSize',10,...
%             'MarkerEdgeColor','red',...
%             'MarkerFaceColor',[0.4 0.5 0.2 ])
%         end
%         xlabel('lateral acceleration [g]')
%         ylabel('longitudinal acceleration [g]')
% %         hold off
%         xlim([-1.1 1.285])
%         ylim([-1.1 1.1])
%     end
    
    

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vx FIGURE:
    figure(2)
    clf(figure(2))
    n = 1;
    subplot(n,1,1), hold on, plot(Vx_Tot,'r','LineWidth',3), ylabel('V_x [m/s]')  , xlabel('Hp'), grid on

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vy FIGURE:
    figure(3)
    clf(figure(3))
    n = 1;
    subplot(n,1,1), hold on, plot(Vy_Tot,'r','LineWidth',3), ylabel('V_y [m/s]')  , xlabel('Hp'), grid on
    
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% W FIGURE:
    figure(4)
    clf(figure(4))
    n = 1;
    subplot(n,1,1), hold on, plot(W_Tot,'r','LineWidth',3), ylabel('\omega [rad/s]')  , xlabel('Hp'), grid on
    
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% STATES FIGURE:
%     clf(figure(1)),figure(1)
%     n = 1;
%     subplot(n,1,1), hold on, plot(XX_dt(1,:),'r','LineWidth',2), ylabel('Vx [m/s]')  , xlabel('Hp'), grid on
% %     subplot(n,1,2), hold on, plot(long_acc), ylabel('Long_{Acc}')  , xlabel('Hp'), grid on
% %     subplot(n,1,3), hold on, plot(lat_acc), ylabel('Lat_{Acc}') , xlabel('Hp'), grid on
%     subplot(n,1,2), hold on, plot(XX_dt(2,:)), ylabel('Vy [m/s]')  , xlabel('Hp'), grid on
%     subplot(n,1,3), hold on, plot(XX_dt(3,:)), ylabel('W [rad/s]') , xlabel('Hp'), grid on
%     subplot(n,1,4), hold on, plot(XX_dt(4,:)), ylabel('E_y [m]')   , xlabel('Hp'), grid on
%     subplot(n,1,5), hold on, plot(XX_dt(5,:)), ylabel('E_{\theta}'), xlabel('Hp'), grid on


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONTROL ACTIONS FIGURE:
    figure(5),
    clf(figure(5)) 
    subplot(2,1,1), plot(Accel_Tot, 'r','LineWidth',3), ylabel('Accel'), grid on
    subplot(2,1,2), hold on, plot(Steer_Tot, 'k','LineWidth',3), ylabel('Steering')
    grid on
    
    
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D TRAJECTORY FIGURE:

if k > 0 && k <= TOT_SIM
    clf(figure(6))
    figure(6)
    hold on, plot(RightSide(1,:),RightSide(2,:),'k', LeftSide(1,:),LeftSide(2,:),'k'),grid on
    %     hold on, plot(RightPB(1,:),RightPB(2,:),':k', LeftPB(1,:),LeftPB(2,:),':k'),grid on
    plot(tXref(1:end-1),tYref(1:end-1),'--k')
    hold on, plot(xp(k,1), yp(k,1),'gs', 'LineWidth',2,...
    'MarkerSize',10, 'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5])
    hold on, plot(xp(k,1:end-1), yp(k,1:end-1), 'r','LineWidth',3);
    hold on, plot(Obs_Pose1(1), Obs_Pose1(2),'gs', 'LineWidth',2,...
    'MarkerSize',10, 'MarkerEdgeColor','k',...
    'MarkerFaceColor',[0.2,0.8,0.5])
    hold on, plot(Obs_Pose2(1), Obs_Pose2(2),'gs', 'LineWidth',2,...
    'MarkerSize',10, 'MarkerEdgeColor','k',...
    'MarkerFaceColor',[0.2,0.8,0.5])
    hold on, plot(xp_R_Obs(k,:), yp_R_Obs(k,:), 'k','LineWidth',3);
    hold on, plot(xp_L_Obs(k,:), yp_L_Obs(k,:), 'k','LineWidth',3);    

    % axis([-5 4 -0.6 3.5])  % axis for 3110 track
%     axis([-3 3 -0.6 3.5])  % axis for oval track
    drawnow
else
    clf(figure(6))
    figure(6)
    hold on, plot(RightSide(1,:),RightSide(2,:),'k', LeftSide(1,:),LeftSide(2,:),'k'),grid on
    %     hold on, plot(RightPB(1,:),RightPB(2,:),':k', LeftPB(1,:),LeftPB(2,:),':k'),grid on
    plot(tXref(1:end-1),tYref(1:end-1),'--k')
    hold on, plot(xp(k,1), yp(k,1),'gs', 'LineWidth',2,...
    'MarkerSize',10, 'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5])
    hold on, plot(xp(k,:), yp(k,:), 'k','LineWidth',3);
    % axis([-5 4 -0.6 3.5])  % axis for 3110 track
%     axis([-3 3 -0.6 3.5])  % axis for oval track
    drawnow
end

% pause(0.5)
    
end


% figure(10), plot(timeElapsed, 'r'), grid on


