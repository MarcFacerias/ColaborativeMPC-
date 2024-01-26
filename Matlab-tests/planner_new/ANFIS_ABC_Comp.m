function [A, B, C] = ANFIS_ABC_Comp(vx, vy, omega, steer, accel, Iden_bestnet)
    
    % data is the input dataset for ANFIS
 
    num_sched_vars  = size(Iden_bestnet.inputs,2);       % Number of inputs (scheduling variables)
    num_MF          = size(Iden_bestnet.inputs(1).mf,2); % Number of membership functions
    num_Rules       = size(Iden_bestnet.Rules,2);        % Number of rules
    
    % Getting parameter matrices:
    A_tmp = zeros(num_Rules,3);
    B_tmp = zeros(num_Rules,2);
    C_tmp = zeros(num_Rules,1);
    
    for i=1:num_Rules
        A_tmp(i,:) = Iden_bestnet.output.mf(i).params(1:3);
        B_tmp(i,:) = Iden_bestnet.output.mf(i).params(4:5);
        C_tmp(i,:) = Iden_bestnet.output.mf(i).params(6);
    end
    
    % Getting MF value:
    M_vx    = zeros(1,2);    
    M_vy    = zeros(1,2);
    M_omega = zeros(1,2);
    M_steer = zeros(1,2);
    M_accel = zeros(1,2);
    
    for i=1:num_MF
        M_vx(i)     = gbellmf( vx,    [Iden_bestnet.input(1).mf(i).params] ); 
        M_vy(i)     = gbellmf( vy,    [Iden_bestnet.input(2).mf(i).params] ); 
        M_omega(i)  = gbellmf( omega, [Iden_bestnet.input(3).mf(i).params] ); 
        M_steer(i)  = gbellmf( steer, [Iden_bestnet.input(4).mf(i).params] ); 
        M_accel(i)  = gbellmf( accel, [Iden_bestnet.input(5).mf(i).params] ); 
    end
    
    % Getting weights:
    weights = zeros(1,num_MF^num_sched_vars);
    counter = 1;
    
	for i=1:num_MF
        for j=1:num_MF
            for k=1:num_MF
                for l=1:num_MF
                    for m=1:num_MF
                        weights(counter) = M_vx(i)*M_vy(j)*M_omega(k)*M_steer(l)*M_accel(m);
                        counter = counter + 1;
                    end
                end
            end
        end
    end
    
    % Normalising weights:
    Norm_weights = zeros(1,num_Rules);
    
    for i=1:num_Rules
        Norm_weights(i) = weights(i) / sum(weights);
    end

    % Computing matrices:
    A = 0;
    B = 0;
    C = 0;
    
    for i=1:num_MF^num_sched_vars
        A = A + Norm_weights(i) * A_tmp(i,:);
        B = B + Norm_weights(i) * B_tmp(i,:);
        C = C + Norm_weights(i) * C_tmp(i);
    end
    
end