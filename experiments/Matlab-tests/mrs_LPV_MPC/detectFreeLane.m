function [ detect ] = detectFreeLane(Obs_Pose, pose,Hp,margin )
    
    detect = 0; 
    for i=1:Hp
        
        distance = sqrt( (pose(1,i)-Obs_Pose(1))^2 + (pose(2,i)-Obs_Pose(2))^2); 

        if distance < margin
            detect = 1; 
            break
        end
    end 
    
end

