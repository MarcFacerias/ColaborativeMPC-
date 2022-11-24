%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 11/07/2019
% Berkeley vehicle model for simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = nonlinear_model(t, states, u, constants, Ew)

    % noise sources (process).
    
        %LATERAL WIND
        % d_air = densidad aire, Cd = coeficiente dinamico
        % Cd = coeficiente dinamico
        % Al = area latereal 
        % vw = velocidad del viento
    
        %Fw = 1/2 * d_air * Cd * Al *vw;
        %dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr - Fw(lf-lr)) / I  ; 
        %dx(2,1) = ( FyF*cos(delta) + FyR - Fw) / m  -  w*vx ; 
        
        %INCLINATION
        % g*sin(alpha)
        
    % 
    %small car
%     lf          = 0.125;
%     lr          = 0.125;
%     m           = 1.98;
%     I           = 0.03;
%     Cf          = 60;
%     Cr          = 60;  
%     mu          = 0.1;
%     g           = 9.8;
    
% %     %large car 
%     Af          = 6.6849;
%     ro_air      = 1.225;
% %     Cd          = 0.29;
%     
%     lf          = 1.52;
%     lr          = 1.22;
%     m           = 1554;
%     I           = 2200;
%     Cf          = 59054;
%     Cr          = 107123;  
%     mu          = 0.1;

    lf          = constants(1);
    lr          = constants(2);
    m           = constants(3);
    I           = constants(4);
    Cf          = constants(5);
    Cr          = constants(6);
    mu          = constants(7);
    g           = constants(8);
    
    a = 0.0;
    b = 0.0;
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
    
     F_drag = mu * vx;
    
    % Equations of motion:
    dx(1,1) = a - F_drag  -  FyF*sin(delta)/m  +  w*vy + Ew(1)*noiseSignal(1);    
    dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx + Ew(2)*noiseSignal(2) ; 
    dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr  ) / I + Ew(3)*noiseSignal(3) ; 
    dx(4,1) = vx*cos(theta) - vy*sin(theta) + Ew(4)*noiseSignal(4);
    dx(5,1) = vx*sin(theta) + vy*cos(theta) + Ew(5)*noiseSignal(5);
    dx(6,1) = w + Ew(6)*noiseSignal(6);
    
end

