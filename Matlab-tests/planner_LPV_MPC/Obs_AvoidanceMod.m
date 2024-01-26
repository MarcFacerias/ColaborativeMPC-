function [ left_limit, right_limit, detect ] = Obs_AvoidanceMod(left_limit, right_limit, Obs_Pose, map, pose, k, Hp, SS_lap, side, side_car )

    [s,~,~] = map.getLocalPosition( pose(1), pose(2), pose(3), side_car);

    [s_Obs, ey_Obs, ~] = map.getLocalPosition( Obs_Pose(1), Obs_Pose(2), 1.16, side  );
    
    security_distance = 0.1;
    
    if ey_Obs < 0
        ey_Obs = ey_Obs + security_distance;
    else
    	ey_Obs = ey_Obs - security_distance; 
    end
    
    max_lim_Ey      = max(right_limit);
    S_Obs_distance  = 2;
    points          = 5;
    ey_steps        = (max_lim_Ey - ey_Obs)/points;
    s_eps           = S_Obs_distance - 0.85 * S_Obs_distance;
    
    end_s = s(end) - SS_lap;
    if end_s < 0
        end_s = 0;
    end
    
    if (s_Obs+s_eps < end_s) && (s_Obs > s(k)-SS_lap) %% Estamos dentro del campo de vision
        
        detect = true; 
        SSS = abs((s(k:end)-SS_lap)-s_Obs);
        Obs_min = min(SSS);
        Obs_indx = find(SSS < Obs_min+0.01);
           
        right_limit(Obs_indx) = ey_Obs;
        sum_Ey = ey_Obs;

        for i=1:points
            sum_Ey = sum_Ey + ey_steps;
            if ( Obs_indx-i > 0 )             
                right_limit(Obs_indx-i) = sum_Ey;                
            end
            right_limit(Obs_indx+i) = sum_Ey;
        end
        
        right_limit  = right_limit(1:Hp+1);
        
    end
    
end

