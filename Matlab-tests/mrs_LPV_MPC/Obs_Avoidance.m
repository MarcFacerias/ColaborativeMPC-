function [ left_limit, right_limit ] = Obs_Avoidance( Obs_Pose, map, s, k, Hp, SS_lap )


    [s_Obs, ey_Obs, etheta_Obs] = map.getLocalPosition( Obs_Pose(1), Obs_Pose(2), 1.16  );
    
    security_distance = 0.1;
    
    if ey_Obs < 0
        ey_Obs = ey_Obs + security_distance;
    else
    	ey_Obs = ey_Obs - security_distance; 
    end
    
    max_lim_Ey      = 0.5;
    S_Obs_distance  = 2;
    points          = 5;
    ey_steps        = (max_lim_Ey - ey_Obs)/points;
    s_eps           = S_Obs_distance - 0.85 * S_Obs_distance;
    
    end_s = s(end) - SS_lap;
    if end_s < 0
        end_s = 0;
    end
    
    if (s_Obs+s_eps < end_s) && (s_Obs > s(k)-SS_lap) %% Estamos dentro del campo de vision
        
        SSS = abs((s(k:end)-SS_lap)-s_Obs);
        Obs_min = min(SSS);
        Obs_indx = find(SSS < Obs_min+0.01);
        
        left_limit = max_lim_Ey * ones(1,Hp+1); 
   
        left_limit(Obs_indx) = ey_Obs;
        sum_Ey = ey_Obs;
        for i=1:points
            sum_Ey = sum_Ey + ey_steps;
            if ( Obs_indx-i > 0 )             
                left_limit(Obs_indx-i) = sum_Ey;                
            end
            left_limit(Obs_indx+i) = sum_Ey;
        end
        
        left_limit  = left_limit(1:Hp+1);
        
    else
        left_limit  = 0.5*ones(1,Hp+1); 
    end
    
    right_limit = -0.5*ones(1,Hp+1);

end

