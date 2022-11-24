
%-------- Compting the gain of the observer using H_\infty design --------- 
%-------------------- M. Pourasghar (25 June 2018)-------------------------
%==========================================================================


function [L] = LQRGainObsv(nx,A,C,Ew,Ev)


Qw = cov(Ew);
Qv = cov(Ev);


gamma = sdpvar(1);
Gamma = sdpvar(nx,nx,'symmetric'); 
P = sdpvar(nx,nx); 

%%
F = [gamma > 0];
F = [F, Gamma > 0];



F11 = -P;
    F12 = (P*A')-(Gamma'*C');
        F13 = P*Ew';
            F14 = Gamma';
                 
F21 = (A*P)-(C*Gamma); 
    F22 = -P;
        F23 = zeros(nx,nx);
            F24 = zeros(nx,nx);
              
           
F31 = Ew*P;
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
    
    
    L = inv(Gamma_sol)*P_sol
    
    eig(A-(L*C))


end