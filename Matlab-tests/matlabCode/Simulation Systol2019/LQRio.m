
%-------- Compting the gain of the observer using H_\infty design --------- 
%-------------------- M. Pourasghar (25 June 2018)-------------------------
%==========================================================================


function [Lio] = LQRio(nx,A,C,Ew,Ev)
% C = [1, 0, 0];
Ew = [.05 0 0;
      0 .05 0;
      0 0 .05];
Ev = [.08];

Q = cov(Ew);
R = cov(Ev);


gamma = sdpvar(1);
Gamma = sdpvar(nx,nx,'symmetric'); 
W = sdpvar(nx,nx); 

%%
F = [gamma >= 0];
F = [F, Gamma >= 0];
%%
F11 = (Gamma*A)+(A'*Gamma)-(W*C)-(C'*W');
    F12 = Gamma*((Q^.5)');
        F13 = W;

                 
F21 = (Q^.5)*Gamma; 
    F22 = -eye(nx,nx);
        F23 = zeros(nx,nx);

              
           
F31 = W';
    F32 = zeros(nx,nx);
        F33 = -R^(-1);

F = [F ,[F11 F12 F13;...
         F21 F22 F23;...
         F31 F32 F33] <= 0];
%%


T11 = gamma * eye(nx,nx);
T12 = eye(nx,nx);

T21 = eye(nx,nx);
T22 = Gamma;


F = [F, [T11, T12; T21, T22] >= 0];


%%


OBJi = gamma;
sdpoptions = sdpsettings('showprogress',1,'solver','sedumi');

solvesdp(F,OBJi,sdpoptions)
solvesdp([F,OBJi*value(OBJi)],[],sdpoptions);

    Gamma_sol  = double(Gamma);  % P solution
    W_sol  = double(W); % W1 solution

    
    gamma_sol = double(gamma);
    
    
    Lio = inv(Gamma_sol)*W_sol;
    
    


end