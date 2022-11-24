%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 09/05/2019
% UPC Driverless vehicle simulation for getting data for learning.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = nonlinear_DriverlessUPC_model(t, states, u)
    
    lf          = 0.902;
    lr          = 0.638;
    m           = 196;
    I           = 93;
    cdA         = 1.64;
    rho         = 1.225;
    g           = 9.81;
    Cf          = 17974;
    Cr          = 24181;


    %% Lateral Pacejka models:
    % ======================== %
    % FRONT PACEJKA PARAMETERS %
    % ======================== %
    Df   = 1.2246e+03;
    Cff   = 1.1804;
    Bf   = -17.3065;
    SHyf  = 0.0014;
    SVyf = 1.9536;

    % ======================= %
    % REAR PACEJKA PARAMETERS %
    % ======================= %
    Dr   = 902.7381;
    Crr   = 1.1804;
    Br   = -19.2720;
    SHyr = 0.0011;
    SVyr = -2.2360;

    
    % States
    vx      = states(1);
    vy      = states(2);
    w       = states(3);


    % Inputs:
    delta   = u(1);
    a       = u(2); 
    road_slope = u(3);  % rad
    v_wind     = u(4);  % m/s
    
    a_F = 0.0;
    a_R = 0.0;

    if abs(vx) > 0.1
        % Front and rear slip angles:
        a_F = atan((vy + lf*w)/vx) - delta;
        a_R = atan((vy - lr*w)/vx);
    end
    
%     if counter < 10
%         % Pacejka tire model:
%         FyF = Df * sin( Cff * atan( Bf * (a_F + SHyf) ) ) + SVyf;
% %         FyR = Dr * sin( Crr * atan( Br * (a_R + SHyr) ) ) + SVyr;
% 
%     else
        % Linear tire model:
        FyF = -Cf * a_F;
        FyR = -Cr * a_R;
%     end
    
    if abs(a_F) > 30.0/180.0*pi || abs(a_R) > 30.0/180.0*pi
        %disp("WARNING: Large slip angles in simulation")
    end
    
    F_drag = 1.4*vx;
    Fy_wind = 0.5 * 1.225 * 1.5 * v_wind^2;
    % % 0.5 * 1.225 * 1.64 * 90^2 * Ts / 196 % Calculo limites perturb
    % % 0.5 * 1.225 * 1.64 * 90^2 * Ts * (0.902 - 0.638) / 93
    
    % Equations of motion:
    dx(1,1) =   a - F_drag  -  FyF*sin(delta)/m  +  w*vy - g*sin(road_slope) ;    
    dx(2,1) = ( FyF*cos(delta) + FyR - Fy_wind ) / m  -  w*vx ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr - Fy_wind*(lf-lr) ) / I  ;
    
%     dx(1,1) =   a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
%     dx(2,1) = ( FyF*cos(delta) + FyR  ) / m  -  w*vx ; 
%     dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr ) / I  ; 
    
    
    
end



% % %%%%%%%%%%%%%%%%%%
% % Pruebas:
% % lf          = 0.902;
% % lr          = 0.638;
% % m           = 196;
% % I           = 93;
% % cdA         = 1.64;
% % rho         = 1.225;
% % g           = 9.81;
% % Cf          = 17974;
% % Cr          = 24181;
% % 
% % Df   = 1.2246e+03;
% % Cff   = 1.1804;
% % Bf   = -17.3065;
% % SHyf  = 0.0014;
% % SVyf = 1.9536;
% % 
% % Dr   = 902.7381;
% % Crr   = 1.1804;
% % Br   = -19.2720;
% % SHyr = 0.0011;
% % SVyr = -2.2360;
% %     
% % a_F = -0.1:0.01:0.1;
% % a_R = -0.1:0.01:0.1;
% % 
% % plot(Df * sin( Cff * atan( Bf * (a_F + SHyf) ) ) + SVyf)
% % hold on, plot(-Cf * a_F)
% % 
% % 
% %         FyR = Dr * sin( Crr * atan( Br * (a_R + SHyr) ) ) + SVyr;


