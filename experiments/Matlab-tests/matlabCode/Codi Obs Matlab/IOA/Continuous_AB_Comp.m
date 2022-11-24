function [ Aobs, Bobs, A, B, Ef, Opseudinv ] = Continuous_AB_Comp( delta, vx, vy, theta, C, option )

%     lf          = 1.0832;
%     lr          = 1.6168;
%     m           = 1448;
%     I           = 2475;
%     Cf          = 98331;
%     Cr          = 120363;  
%     mu          = 1;
    
%     lf          = 1.52;
%     lr          = 1.22;
%     m           = 1554;
%     I           = 2200;
%     Cf          = 59054;
%     Cr          = 107123;  
%     mu          = 0.1;
   
    
    lf          = 0.125;
    lr          = 0.125;
    m           = 1.98;
    I           = 0.03;
    Cf          = 60;
    Cr          = 60;  
    mu          = 0.1;
    dt          = 0.005;
    
    if option == 'Invariant B'
        
        B = [ 0      1 ;	%[delta accel]
             Cf/m    0 ; 
             lf*Cf/I 0 ;
              0      0 ;
              0      0 ;
              0      0]; 
         
    elseif option == 'Varianttt B'
        
        B = [ -Cf*sin(delta)/m      1 ;     %[delta accel]
              Cf*cos(delta)/m       0 ; 
              lf*Cf*cos(delta)/I    0 ;
                    0               0 ;
                    0               0 ;
                    0               0];
    end
     

    A11 =  -mu;
%     A11 =  0; 
    A12 = (sin(delta) * Cf) / (m*vx);
    A13 = (sin(delta) * Cf * lf) / (m*vx) + vy;
    A22 = -(Cr + Cf * cos(delta)) / (m*vx);
    A23 = -(lf * Cf * cos(delta) - lr * Cr) / (m*vx) - vx;
    A32 = -(lf * Cf * cos(delta) - lr * Cr) / (I*vx);
    A33 = -(lf * lf * Cf * cos(delta) + lr * lr * Cr) / (I*vx);

    A = [ A11               A12     A13  0  0  0 ;  %[vx]
           0                A22     A23  0  0  0 ;  %[vy]
           0                A32     A33  0  0  0 ;  %[wz]  
           cos(theta)  -sin(theta)  0    0  0  0 ;
           sin(theta)   cos(theta)  0    0  0  0 ;
           0                0       1    0  0  0 ;];

 
    Ef         = [-0.1;
                  0;
                  0;
                  0;
                  0;
                  0];           % Perturbation vector: Drag force made by the wind over the vehicle.
    
    Opseudinv   = pinv(C*Ef);                 
    
    Aobs = (eye(6) - Ef*Opseudinv*C) * (eye(6)+A*dt);
    Bobs = (eye(6) - Ef*Opseudinv*C) * B*dt;
    
end




