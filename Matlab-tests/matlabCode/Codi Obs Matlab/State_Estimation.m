function [ x_est ] = State_Estimation( x_est, y_meas, Accel, Steer, C_obs, index, Llmis, Ts, SchedVars_Limits)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %\\ Vehicle states observation:
    %   x_est: [ vx vy omega ]
    %   y_meas: Real Vehicle Output (states that can be measured)
    %   Accel: Acceleration control action
    %   Steer: Steering angle control action
    %   C_obs: Output matrix
    %   index: control loop index
    %   Ts: Sample time for the dynamic control loop (1 ms)
    %   L: Observer gain [8 vertexes]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if(index<5)
        Vx      = y_meas(1);
        Vy      = y_meas(2);
        Theta   = y_meas(5);
    else
        Vx      = x_est(1);
        Vy      = x_est(2);
        Theta   = x_est(6);
    end
   
    %     [ A_obs, B_obs, A, B, Ef, Opseudinv ] = Continuous_AB_Comp( Steer, Vx, Vy, C_obs, 'Invariant B' );
    [ A_obs, B_obs, A, B, Ef, Opseudinv ] = Continuous_AB_Comp( Steer, Vx, Vy, Theta, C_obs, 'Varianttt B'); 
                            
    L_gain  = K_Computation( Steer, Vx, Vy, Theta, SchedVars_Limits, Llmis );

    AO = ( A_obs + L_gain * C_obs );
    
    
    % State estimation:
    x_est  =  x_est    + Ts * AO        * x_est... 
                       + Ts * B_obs     * [Steer; Accel]...
                       - Ts * L_gain    * y_meas;
                       %+ Ef * Opseudinv * y_meas(:,index)...
                       %- Ef * Opseudinv * y_meas(:,index-1);

    % f_est(index) = pinv(Ts*C_obs*Ef) * ( y(:,index) - ( (eye(3)+Ts*A)*C_obs*x_est(:,index-1) + Ts*C_obs*B*[Force;Steer] ) );            

end

