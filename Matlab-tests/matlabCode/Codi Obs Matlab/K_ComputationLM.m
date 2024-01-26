function [ K_gain ] = K_ComputationLM( steer, vx, vy, theta, theta_lm, SchedVars_Limits, Klmi )     

    
    M_vx_despl_min      = (SchedVars_Limits(1,2) - vx) / (SchedVars_Limits(1,2) - SchedVars_Limits(1,1) );
    M_vy_despl_min      = (SchedVars_Limits(2,2) - vy) / (SchedVars_Limits(2,2) - SchedVars_Limits(2,1) );
    M_steer_min         = (SchedVars_Limits(4,2) - steer) / (SchedVars_Limits(4,2) - SchedVars_Limits(4,1)); 
    M_theta_min         = (SchedVars_Limits(6,2) - theta) / (SchedVars_Limits(6,2) - SchedVars_Limits(6,1)); 

    mu(1)               = M_vx_despl_min     * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
    mu(2)               = M_vx_despl_min     * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
    mu(3)               = M_vx_despl_min     * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
    mu(4)               = M_vx_despl_min     * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
    mu(5)               = M_vx_despl_min     * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
    mu(6)               = M_vx_despl_min     * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
    mu(7)               = M_vx_despl_min     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
    mu(8)               = M_vx_despl_min     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);
    
    mu(9)               = (1-M_vx_despl_min)     * M_vy_despl_min      * M_steer_min      *  M_theta_min; 
    mu(10)              = (1-M_vx_despl_min)     * M_vy_despl_min      * M_steer_min      *  (1-M_theta_min); 
    mu(11)              = (1-M_vx_despl_min)     * M_vy_despl_min      * (1-M_steer_min)  *  M_theta_min; 
    mu(12)              = (1-M_vx_despl_min)     * M_vy_despl_min      * (1-M_steer_min)  *  (1-M_theta_min);
    mu(13)              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * M_steer_min      *  M_theta_min; 
    mu(14)              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * M_steer_min      *  (1-M_theta_min);
    mu(15)              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  M_theta_min; 
    mu(16)              = (1-M_vx_despl_min)     * (1-M_vy_despl_min)  * (1-M_steer_min)  *  (1-M_theta_min);  

    K_gain = zeros(length(Klmi(:,:,1)), 5 );
    for i=1:length(mu)
        K_gain = K_gain + mu(i)*Klmi(:,:,i);
    end

end

