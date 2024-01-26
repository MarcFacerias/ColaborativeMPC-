function [ Aobs, Bobs, A, B, Ef, Opseudinv ] = Observer_Computation( DELTA, V, ALPHA, C )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function computes the current model matrices A and B for state
% estimation.
% DELTA represents the steering angle
% V is the linear vehicle velocity
% ALPHA represents the slip angle of the vehicle
% C is the output matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
    m       = vehicle.m;
    I       = vehicle.I;
    a       = vehicle.a;
    b       = vehicle.b;
    Cf      = vehicle.Cf;
    Cr      = vehicle.Cr;
    ro      = vehicle.ro;
    Cd      = vehicle.Cd;
    Area    = vehicle.Area;
    g       = vehicle.g;
    mu_roz  = vehicle.mu_roz;

    % Fuerzas que se oponen
    F_rozamiento = mu_roz*m*g;              % This force is going to be estimated
    F_drag       = 0.5*ro*Cd*Area*V*V;      % Drag force made by the vehicle trough the air.
    
    
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    % MODELO SIMPLIFICADO:
    % %         %A1  = -(F_drag+F_rozamiento)/(m*V);
    % %         A1  = -(F_drag)/(m*V);
    % %         A2  = Cf*sin(DELTA)/m;
    % %         A3  = Cf*sin(DELTA)*a/(m*V);
    % %         A4  = -(Cf*cos(DELTA)+Cr)/(m*V);
    % %         A5  = -(Cf*a*cos(DELTA)-Cr*b)/(m*V*V) - 1;
    % %         A6  = (Cr*b-Cf*a*cos(DELTA))/I;
    % %         A7  = (-Cf*a*a*cos(DELTA) + Cr*b*b)/(I*V);
    % %         A8  = 1/m;
    % %         A9  = Cf*sin(DELTA)/m;
    % %         A10 = Cf*cos(DELTA)/(m*V);
    % %         A11 = Cf*a*cos(DELTA)/I;
    % %     
    % %         A = [A1 A2 A3; 
    % %              0 A4 A5; 
    % %              0 A6 A7];
    % %     
    % %         B = [A8 A9;
    % %              0 A10; 
    % %              0 A11];
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% % % % %     MODELO MAS COMPLETO
% % % % %     ESTO TODAVIA HAY QUE PROBARLO:
% % % %     A1 = -(F_drag+F_rozamiento) / (m*V);
% % % %     A2 = Cf*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)-sin(ALPHA)) / m;
% % % %     A3 = Cf*(a*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) - b*sin(ALPHA)) / (m*V);        
% % % %     B1 = Cf*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) / m;        
% % % %     B2 = cos(ALPHA) / m;
% % % %     A4 = -Cf*(cos(ALPHA)*cos(DELTA)+sin(ALPHA)*sin(DELTA)+cos(ALPHA)) / (m*V);
% % % %     A5 = ( (-Cf*a*(cos(DELTA)*cos(ALPHA)+sin(ALPHA)*sin(DELTA)) + Cf*b*cos(ALPHA)) / (m*(V*V)) ) - 1;        
% % % %     B3 = Cf*(cos(ALPHA)*cos(DELTA)+sin(ALPHA)*sin(DELTA)) / (m*V);    
% % % %     B4 = -sin(ALPHA) / (m*V);    
% % % %     A6 = Cf*(b-a*cos(DELTA)) / I;
% % % %     A7 = -Cf*(a*a*cos(DELTA) + b*b) / (I*V);    
% % % %     B5 = (Cf*a*cos(DELTA)) / I;
% % % % 
% % % %     A = [A1 A2 A3; 
% % % %          0 A4 A5; 
% % % %          0 A6 A7];
% % % %     B = [ B2 B1;
% % % %           B4 B3;
% % % %           0  B5];
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    
    A1 = -(F_drag+F_rozamiento) / (m*V);
%     A1 = -(F_drag) / (m*V);
    A2 = Cf*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)-sin(ALPHA)) / m;
    A3 = Cf*(a*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) - b*sin(ALPHA)) / (m*V);        
    A4 = -Cf*(cos(ALPHA)*cos(DELTA)+sin(ALPHA)*sin(DELTA)+cos(ALPHA)) / (m*V);
    A5 = ( (-Cf*a*(cos(DELTA)*cos(ALPHA)+sin(ALPHA)*sin(DELTA)) + Cf*b*cos(ALPHA)) / (m*(V*V)) ) - 1;         
    A6 = Cf*(b-a*cos(DELTA)) / I;
    A7 = -Cf*(a*a*cos(DELTA) + b*b) / (I*V);    
   
    A = [A1 A2 A3; 
         0 A4 A5; 
         0 A6 A7];
         
    B1 = Cf*(sin(0)*cos(0)-sin(0)*cos(0)) / m;               
    B2 = cos(0) / m;
    B3 = Cf*(cos(0)*cos(0)+sin(0)*sin(0)) / (m*5);    
    B4 = -sin(0) / (m*5);    
    B5 = (Cf*a*cos(0)) / I;
    
    B = [ B2 B1;
          B4 B3;
          0  B5];    
    
    

    %% Observer parts:

    Ef         = [-1/m;
                  0;
                  0];           % Perturbation vector: Drag force made by the wind over the vehicle.
    
    Opseudinv   = pinv(C*Ef);                 
    
    Aobs = (eye(3) - Ef*Opseudinv*C)*A;
    Bobs = (eye(3) - Ef*Opseudinv*C)*B;
    
end

