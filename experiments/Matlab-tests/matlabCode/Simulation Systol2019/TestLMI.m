clc, clear all, close all;

nx = 3; ny = 1; nu = 2;
%-------------------------------------------------------------------------
%% Continuous-time state-space model 

Ac = [-0.02876264232, 0, 0; 
       0.02876264232, -0.03618912464, 0.007426482315; 
       0, 0.007426482315, -0.01228825622];
  
Bc = [64.93506494, 0; 0, 0;0, 0];
Cc = [1, 0, 0;0, 1, 0; 0 0 1];
% Cc = [1, 0, 0];

u = .1e-3*[2 0]';


%% Discrete-time state-space model 
Ts = 1;
A = eye(3,3)+(Ts*Ac); % pq suma la identidad? 
Bu = Ts*Bc;
C = Cc;



%% Obsrever gain
% L_poles = [0.3;0.9;0.2];
% L = place(A',C', L_poles).';
%% Direction of the uncertainties

%vectors que te diuen com afecta el noise a la resta del sistema

Ew = .0001*[.05 0 0;
      0 .05 0;
      0 0 .05];
Ev = [.08 0 0;
      0 .08 0;
      0 0 .08];
  
  
  
  
%%
% Cov = covariance -> covariance with itself equivalent a la variança
Qw = cov(Ew);
Qv = cov(Ev);

gamma = sdpvar(1);
Gamma = sdpvar(nx,nx); 
P = sdpvar(nx,nx); 

% Hi ha dos gammes al calcul del LMI (?)
F = [gamma > 0];
F = [F, Gamma > 0];

% definim el LMI que necesitarem -> depen de la covariance del soroll ->
% Qun LMI es aquest? 

F11 = -P;
    F12 = (P*A')-(Gamma'*C');
        F13 = P*Qw';
            F14 = Gamma';
                 
F21 = (A*P)-(C*Gamma); 
    F22 = -P;
        F23 = zeros(nx,nx);
            F24 = zeros(nx,nx);
              
           
F31 = Qw*P;
    F32 = zeros(nx,nx);
        F33 = -ones(nx,nx);
            F24 = zeros(nx,nx);

F41 = Gamma;
    F42 = zeros(nx,nx);
        F43 = zeros(nx,nx);
            F44 = -inv(Qv);            
            
            
            
        
F = [F ,[F11 F12 F13 F14;...
         F21 F22 F23 F24;...
         F31 F32 F33 F24;...
         F41 F42 F43 F44] < 0];




OBJi = gamma;
sdpoptions = sdpsettings('showprogress',1,'solver','sedumi');

solvesdp(F,OBJi,sdpoptions)
solvesdp([F,OBJi*value(OBJi)],[],sdpoptions);

    Gamma_sol  = double(Gamma);  % P solution
    P_sol  = double(P); % W1 solution

    
    gamma_sol = double(gamma);
    
    
    L = inv(Gamma_sol)*P_sol % observer gain
    
    eig(A-(L*C)) %observer stability















