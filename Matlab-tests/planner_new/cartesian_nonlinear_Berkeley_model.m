%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 12/03/2019
% UC Berkeley vehicle simulation for getting data for learning.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = cartesian_nonlinear_Berkeley_model(t, states, u)
    
 	lf = 0.125;
    lr = 0.125;
    m  = 1.98;
    I  = 0.03;
    mu = 0.05;
    
    num_states = 5;
    
    if (num_states == 3)
        vx      = states(1);
        vy      = states(2);
        w       = states(3);
        
    elseif (num_states == 5)
        vx      = states(1);
        vy      = states(2);
        w       = states(3);
        ey      = states(4);      
        epsi    = states(5);      
    else
        vx      = states(1);
        vy      = states(2);
        w       = states(3);
        x       = states(4);      
        y       = states(5);     
        theta   = states(6); 
    end
    

    % Inputs:
    delta   = u(1);
    a       = u(2); 
    curv    = u(3);
    
    a_F = 0.0;
    a_R = 0.0;

    if abs(vx) > 0.1
        a_F = atan((vy + lf*w)/vx) - delta;
        a_R = atan((vy - lr*w)/vx);
        % a_F = delta - atan((vy + lf*w)/vx) ;
        % a_R = atan((-vy + lr*w)/vx);
    end
    
%     FyF = -pacejka_tire_model(a_F);
%     FyR = -pacejka_tire_model(a_R);
    FyF = -65 * a_F;
    FyR = -65 * a_R;
    
    if abs(a_F) > 30.0/180.0*pi || abs(a_R) > 30.0/180.0*pi
        disp("WARNING: Large slip angles in simulation")
    end
    
    F_drag = mu*vx;
    
    if (num_states == 3)
        dx(1,1) =   a - F_drag  -  FyF*sin(delta)/m  +  w*vy;    
        dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx; 
        dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr ) / I; 
        
    elseif (num_states == 5)
        dx(1,1) =   a - F_drag  -  FyF*sin(delta)/m  +  w*vy ;    
        dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx ; 
        dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr ) / I  ; 
        dx(4,1) = vx*sin(epsi)+vy*cos(epsi);                              %[ey] 
        dx(5,1) = w - ( (vx*cos(epsi)-vy*sin(epsi))*curv / (1-ey*curv) ); %[etheta = epsi]
        
    else
        dx(1,1) =   a - F_drag  -  FyF*sin(delta)/m  +  w*vy;    
        dx(2,1) = ( FyF*cos(delta) + FyR ) / m  -  w*vx; 
        dx(3,1) = ( FyF*lf*cos(delta) - FyR*lr ) / I; 
        dx(4,1) = vx*cos(theta) - vy*sin(theta);    %[x]
        dx(5,1) = vx*sin(theta) + vy*sin(theta);    %[y]
        dx(6,1) = w;                                %[theta]  
    end     
    
end
