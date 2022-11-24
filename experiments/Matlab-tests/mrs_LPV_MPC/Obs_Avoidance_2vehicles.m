function [ left_limit, right_limit ] = Obs_Avoidance_2vehicles( Obs_Pose1, Obs_Pose2, map, s, k, Hp, SS_lap )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Siempre el coche 1 estará en la parte izq. de la pista y el coche 2 en la
% parte derecha de la pista.
% The output is the lateral error profile bounded in [-0.5, 0.5]m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update limits of the track
% Case where we have a car ahead?
% Common parameters:
    max_lim_Ey          = 0.5; % maximum amount we can dimish the track limit
    S_Obs_distance      = 2; % distance at which the obstable is detected? 
    points              = 7;
    security_distance   = 0.1; % meters
    
    % pose to track coordinates (modeled as a box)
    [s_Obs1, ey_Obs1, etheta_Obs1] = map.getLocalPosition( Obs_Pose1(1), Obs_Pose1(2), 1.16  );
    [s_Obs2, ey_Obs2, etheta_Obs2] = map.getLocalPosition( Obs_Pose2(1), Obs_Pose2(2), 1.16  );
    
    % enlarge the box
    ey_Obs1 = ey_Obs1 - security_distance; 
    ey_Obs2 = ey_Obs2 + security_distance; % ?? 
    
    % steps to smooth the transition in terms of both s and lateral
    % distances 
    ey_steps        = (max_lim_Ey - ey_Obs1)/points;
    s_eps           = S_Obs_distance - 0.85 * S_Obs_distance; % some kind of safety margin?
    
    % check of lap reset
    end_s = s(end) - SS_lap;
    if end_s < 0
        end_s = 0;
    end
    
    % i guess we do two checks depending on the side at which the obstacle
    % is ?
    % Left side (car 1):
    if (s_Obs1+s_eps < end_s) && (s_Obs1 > s(k)-SS_lap) %% Estamos dentro del campo de vision
        
        SSS = abs((s(k:end)-SS_lap)-s_Obs1);
        Obs_min = min(SSS);
        Obs_indx = find(SSS < Obs_min+0.01);
        
        left_limit = max_lim_Ey * ones(1,Hp+1); 
   
        left_limit(Obs_indx) = ey_Obs1;
        sum_Ey = ey_Obs1;
        for i=1:points
            sum_Ey = sum_Ey + ey_steps;     % suma
            if ( Obs_indx-i > 0 )             
                left_limit(Obs_indx-i) = sum_Ey;                
            end
            left_limit(Obs_indx+i) = sum_Ey;
        end
        
        left_limit  = left_limit(1:Hp+1);
        
    else
        left_limit  = 0.5*ones(1,Hp+1); 
    end
    
    
    % Right side (car 2):
    if (s_Obs2+s_eps < end_s) && (s_Obs2 > s(k)-SS_lap) %% Estamos dentro del campo de vision
        
        SSS = abs((s(k:end)-SS_lap)-s_Obs2);
        Obs_min = min(SSS);
        Obs_indx = find(SSS < Obs_min+0.01);
        
        right_limit = -max_lim_Ey * ones(1,Hp+1); 
   
        right_limit(Obs_indx) = ey_Obs2;
        sum_Ey = ey_Obs2;
        for i=1:points
            sum_Ey = sum_Ey - ey_steps;         % resta
            if ( Obs_indx-i > 0 )             
                right_limit(Obs_indx-i) = sum_Ey;                
            end
            right_limit(Obs_indx+i) = sum_Ey;
        end
        
        right_limit  = right_limit(1:Hp+1);
        
    else
        right_limit  = -0.5*ones(1,Hp+1); 
    end
end

