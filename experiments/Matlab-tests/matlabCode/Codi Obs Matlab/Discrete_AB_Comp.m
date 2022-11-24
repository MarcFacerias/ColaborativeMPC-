function [ Aobs, Bobs, A, B, Ef, Opseudinv ] = Discrete_AB_Comp( delta, vx, vy, Ts, C, option )

%     lf          = 0.902;
%     lr          = 0.638;
%     m           = 196;
%     I           = 93;
%     Cf          = 17974;
%     Cr          = 24181;  
%     mu          = 1.4;
    
    lf          = 0.125;
    lr          = 0.125;
    m           = 1.98;
    I           = 0.03;
    Cf          = 60;
    Cr          = 60;  
    mu          = 0.5;
    
    if option == 'Invariant B'
        
        B = [ 0      1 ;	%[delta accel]
             Cf/m    0 ; 
             lf*Cf/I 0 ];
         
    elseif option == 'Varianttt B'
        
        B = [ -Cf*sin(delta)/m      1 ;     %[delta accel]
              Cf*cos(delta)/m       0 ; 
              lf*Cf*cos(delta)/I    0 ];     
    end
     
    B = Ts * B;

    A11 =  -mu;   
    A12 = (sin(delta) * Cf) / (m*vx);
    A13 = (sin(delta) * Cf * lf) / (m*vx) + vy;
    A22 = -(Cr + Cf * cos(delta)) / (m*vx);
    A23 = -(lf * Cf * cos(delta) - lr * Cr) / (m*vx) - vx;
    A32 = -(lf * Cf * cos(delta) - lr * Cr) / (I*vx);
    A33 = -(lf * lf * Cf * cos(delta) + lr * lr * Cr) / (I*vx);

    A = [ A11  A12  A13  ;  %[vx]
           0   A22  A23  ;  %[vy]
           0   A32  A33 ];  %[wz]  
       
    A = eye(3) + Ts * A;

    
  	%% Observer parts:

    Ef         = [-1/m;
                  0;
                  0];           % Perturbation vector: Drag force made by the wind over the vehicle.
    
    Opseudinv   = pinv(C*Ef);                 
    
    Aobs = (eye(3) - Ef*Opseudinv*C)*A;
    Bobs = (eye(3) - Ef*Opseudinv*C)*B;


    
    
    %%

% %     if option == 'Invariant B'
% %         
% %         B = [ 0      1 ;	%[delta accel]
% %              Cf/m    0 ; 
% %              lf*Cf/I 0 ;
% %               0      0 ;
% %               0      0]; 
% %          
% %     elseif option == 'Varianttt B'
% %         
% %         B = [ -Cf*sin(delta)/m      1 ;     %[delta accel]
% %               Cf*cos(delta)/m       0 ; 
% %               lf*Cf*cos(delta)/I    0 ;
% %                      0              0 ;
% %                      0              0];     
% %     end
% %      
% %     B = Ts * B;
% % 
% %     A11 =  -mu;   
% %     A12 = (sin(delta) * Cf) / (m*vx);
% %     A13 = (sin(delta) * Cf * lf) / (m*vx) + vy;
% %     A22 = -(Cr + Cf * cos(delta)) / (m*vx);
% %     A23 = -(lf * Cf * cos(delta) - lr * Cr) / (m*vx) - vx;
% %     A32 = -(lf * Cf * cos(delta) - lr * Cr) / (I*vx);
% %     A33 = -(lf * lf * Cf * cos(delta) + lr * lr * Cr) / (I*vx);
% % 
% %     A = [ A11  A12  A13   0   0;  %[vx]
% %            0   A22  A23   0   0;  %[vy]
% %            0   A32  A33   0   0;  %[wz]  
% %           -1    0    0    0   0;  %[vx_integral]
% %            0    0   -1    0   0]; %[wz_integral]
% %        
% %     A = eye(5) + Ts * A;

end

