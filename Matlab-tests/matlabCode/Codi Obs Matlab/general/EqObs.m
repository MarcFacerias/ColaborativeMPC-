    syms vx vy w x y theta a delta lf lr m I Cf Cr mu g real;


    A11 =  0; 
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
     
       
   B = [ -Cf*sin(delta)/m      1 ;     %[delta accel]
          Cf*cos(delta)/m       0 ; 
          lf*Cf*cos(delta)/I    0 ;
                0               0 ;
                0               0 ;
                0               0];
            
   sx = [vx vy w x y theta]';
   u  = [delta a]';
   
   eq = A*sx + B*u;
   simplify(eq)
   