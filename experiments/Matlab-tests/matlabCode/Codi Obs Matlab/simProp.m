% Test Observer
% data  =[states(6),acc,steer]
% initialization
% addpath '/home/marc/IRI_Internship/tfm/matlabCode/Codi Obs Matlab'
MAIN_Observer_MOD();
data = readNPY("experimentObs.npy");

dt = 0.033;
[N,~] = size(data);
y = [];

est_shape = eye(6);

Ew             = [ 0 0 0 0 0 0; 
                   0 0 0 0 0 0;
                   0 0 0 0 0 0;
                   0 0 0 0 0 0;
                   0 0 0 0 0 0;
                   0 0 0 0 0 0];
              
Ev             = [ 0 0 0 0 0 0; 
                   0 0 0 0 0 0;
                   0 0 0 0 0 0;
                   0 0 0 0 0 0;
                   0 0 0 0 0 0];

C              = [ 1 0 0 0 0 0; 
                   0 0 1 0 0 0;
                   0 0 0 1 0 0;
                   0 0 0 0 1 0;
                   0 0 0 0 0 1];
              
        
for i=3:N
    
   y          = data(i-1,[1,3,4,5,6]);
   u          = data(i-1,[7,8]);
   
   vx         = data(i,1);
   vy         = data(i,2);
   steer      = data(i,8);
   theta      = data(i,6);
   
   [ L ] = K_Computation( steer, vx, vy, theta, SchedVars_Limits, Llmi ); 
   
   [ ~, ~, A, B, ~, ~ ] = Continuous_AB_Comp( steer, ...
       vx, vy, theta, C, 'Varianttt B' );
   
   
   
   A_obs = eye(6) + dt*A;
   B_obs = dt*B;
   
   shape_priori = [A*est_shape,Ew];
   est_shape = [eye(6) - L*C, shape_priori, -L*Ev];
       
end