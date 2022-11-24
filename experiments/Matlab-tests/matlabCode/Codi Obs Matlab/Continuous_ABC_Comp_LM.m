function [ Aobs, Bobs, A, B,C, Ef, Opseudinv ] = Continuous_ABC_Comp_LM( delta, vx, vy, theta, theta_lm, option )
     
    
    lf          = 0.125;
    lr          = 0.125;
    m           = 1.98;
    I           = 0.03;
    Cf          = 60;
    Cr          = 60;  
    mu          = 0.1;
    dt          = 0.005;
    covTH       = 0.1;
    
    lambda      = 1 - exp(-covTH^2) + exp(-covTH^2/2);
    
    if option == 'Invariant B'
        
        B = [ 0      1 ;	%[delta accel]
             Cf/m    0 ; 
             lf*Cf/I 0 ;
              0      0 ;
              0      0 ;
              0      0 ;
              0      0 ;
              0      0]; 
         
    elseif option == 'Varianttt B'
        
        B = [ -Cf*sin(delta)/m      1 ;     %[delta accel]
              Cf*cos(delta)/m       0 ; 
              lf*Cf*cos(delta)/I    0 ;
                    0               0 ;
                    0               0 ;
                    0               0 ;
                    0               0 ;
                    0               0];
    end
     

%   A11 =  -mu; 
    A11 = 0;
    A12 = (sin(delta) * Cf) / (m*vx);
    A13 = (sin(delta) * Cf * lf) / (m*vx) + vy;
    A22 = -(Cr + Cf * cos(delta)) / (m*vx);
    A23 = -(lf * Cf * cos(delta) - lr * Cr) / (m*vx) - vx;
    A32 = -(lf * Cf * cos(delta) - lr * Cr) / (I*vx);
    A33 = -(lf * lf * Cf * cos(delta) + lr * lr * Cr) / (I*vx);

    A = [ A11               A12     A13  0  0  0  0  0;  %[vx]
           0                A22     A23  0  0  0  0  0;  %[vy]
           0                A32     A33  0  0  0  0  0;  %[wz]  
           cos(theta)  -sin(theta)  0    0  0  0  0  0;
           sin(theta)   cos(theta)  0    0  0  0  0  0;
           0                0       1    0  0  0  0  0;
           0                0       0    0  0  0  1  0;
           0                0       0    0  0  0  0  1;];
           
    c64 = -lambda*cos(theta_lm);
    c65 = -lambda*sin(theta_lm);
    c67 = -lambda*sin(theta_lm);
    c68 = lambda*cos(theta_lm);
    
    c74 = lambda*cos(theta_lm);
    c75 = lambda*sin(theta_lm);
    c77 = lambda*sin(theta_lm);
    c78 = -lambda*cos(theta_lm);
    
    C =   [  1 0 0 0   0   0 0   0; %equivalent to Hx
             0 0 1 0   0   0 0   0;
             0 0 0 1   0   0 0   0;
             0 0 0 0   1   0 0   0; 
             0 0 0 0   0   1 0   0;
             0 0 0 c64 c65 0 c67 c68;                
             0 0 0 c74 c75 0 c77 c78;
             0 0 0 0   0   1 0   0];

 
    Ef         = [-mu;
                  0;
                  0;
                  0;
                  0;
                  0];           % Perturbation vector: Drag force made by the wind over the vehicle.
    
    Opseudinv   = pinv(C*Ef);                 
    
    Aobs = (eye(6) - Ef*Opseudinv*C)*(eye(6) + A*dt);
    Bobs = (eye(6) - Ef*Opseudinv*C)*B*dt;
    
end




