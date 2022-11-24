function [ controller ] = LPV_MPC_fnc_dt_Vnew( Hp, dt, varargin )

% control actions 
x       = sdpvar(5,Hp+1,'full');    % [vx, vy, w, ey, etheta, t] 5xN+1
% placeholder for the computed u
u       = sdpvar(2,Hp,'full');      % [delta, acceleration] 2xN+1
% placeholder for the initial control action
pastu   = sdpvar(2, 1,'full'); 
%Track info 1xN+1
curv    = sdpvar(1,Hp+1,'full'); % curvature
sc      = sdpvar(1,Hp+1,'full'); % traversed distance
sc_V    = sdpvar(1,Hp+1,'full'); 

% limits 
left_limit  = sdpvar(1,Hp+1,'full');
right_limit = sdpvar(1,Hp+1,'full');

%A matrix (LPV)
A_1     = sdpvar(1,Hp,'full'); A_5     = sdpvar(1,Hp,'full');
A_2     = sdpvar(1,Hp,'full'); A_6     = sdpvar(1,Hp,'full');
A_3     = sdpvar(1,Hp,'full'); A_7     = sdpvar(1,Hp,'full');
A_4     = sdpvar(1,Hp,'full'); A_8     = sdpvar(1,Hp,'full');
A_9     = sdpvar(1,Hp,'full'); A_10    = sdpvar(1,Hp,'full');

%B matrix (LPV)
B_1     = sdpvar(1,Hp,'full'); B_3     = sdpvar(1,Hp,'full');
B_2     = sdpvar(1,Hp,'full'); B_4     = sdpvar(1,Hp,'full');

% TUNNING: 
% max_Vel_X   = 3.8;    % max. L-shape

try
    max_Vel_X = varargin{1}; 
catch
    max_Vel_X   = 3.5;    % max. oval & 3110
end
%acceleration limits 
max_a_lat   = 1.5;
max_a_long  = 0.7; 
min_a_long  = -1.7;


%  Tunning matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
QQ  = -diag( [0.000000000213635, -0.000000000000088, -9.703658572659423, -0.153591566469547] );   

% LL  = -[2.76633969806571 1.00702414775175 0.187661946033823 -0.0329493219494661]; 
LL  = -[0.0 1.00702414775175 0.187661946033823 -0.0329493219494661]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  


% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % NEW ONE:
% % QQ = [  -0.445908807869696   0                   0                   0;
% %                    0  -0.017490528523361                   0                   0;
% %                    0                   0  -0.851860539011633                   0;
% %                    0                   0                   0  -0.153742367356473];
% % 
% % LL = [-0.000000000000000   0.000007260449747   0.950687440813111   2.128225256917406]; 
% % 
% % % q = 100;
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%% Build the optimisation problem see eq 14 in Euge planning paper 
constraints = [];
objective = 0;

for k = 1:Hp

%     objective = objective...
%         + [x(4,k)*curv(k); x(1,k); x(2,k); x(5,k)]' *QQ* [x(4,k)*curv(k); x(1,k); x(2,k); x(5,k)] ...
%         + LL*[0; x(1,k); x(2,k); x(5,k)] + 200*sc(k)^2 +
%         x(3,k)*0.002*x(3,k);

    %     objective = objective...
    %         + [x(4,k)*curv(k); x(1,k); x(2,k); x(5,k)]' *QQ* [x(4,k)*curv(k); x(1,k); x(2,k); x(5,k)] ...
    %         + LL*[x(4,k)*curv(k); x(1,k); x(2,k); x(5,k)]; % + 200*sc(k)^2 + x(3,k)*0.002*x(3,k);

        objective = objective...
            + [x(4,k); x(1,k); x(2,k); x(5,k)]' *QQ* [x(4,k); x(1,k); x(2,k); x(5,k)] ...
            + LL*[x(4,k); x(1,k); x(2,k); x(5,k)] + 200*sc(k)^2 + x(3,k)*0.05*x(3,k);

%         objective = objective...
%             + [x(4,k); x(1,k); x(2,k); x(5,k)]' *QQ* [x(4,k); x(1,k); x(2,k); x(5,k)] ...
%             + LL*[x(4,k); x(1,k); x(2,k); x(5,k)] + 200*sc(k)^2 + 5*((x(3,k+1)-x(3,k))/dt);

    constraints = [constraints;
        % vx:
        x(1,k+1) == x(1,k) + ( -1*x(1,k) + A_5(1,k)*x(2,k)  +  A_6(1,k)*x(3,k)   +  B_1(1,k)*u(1,k)  +  B_2(1,k)*u(2,k) ) * dt; 
        % vy:
        x(2,k+1) == x(2,k) + ( A_7(1,k)*x(2,k)  +  A_8(1,k)*x(3,k)   +  B_3(1,k)*u(1,k) ) * dt;  
        % w:
        x(3,k+1) == x(3,k) + ( A_9(1,k)*x(2,k)  +  A_10(1,k)*x(3,k)  +  B_4(1,k)*u(1,k) ) * dt;  
        % ey:
        x(4,k+1) == x(4,k) + ( A_4(1,k)*x(5,k)  +  A_3(1,k) ) * dt;          
        % etheta:
        x(5,k+1) == x(5,k) + ( -A_1(1,k)*curv(k)*x(1,k)  + A_1(1,k)*A_2(1,k)*curv(k)*x(2,k)  +  x(3,k) ) * dt;       
        
        left_limit-sc(k)<= x(4,k)                   <= right_limit + sc(k);        
        0.9         <= x(1,k)                   <= max_Vel_X; 
        min_a_long  <= ((x(1,k+1)-x(1,k))/dt)   <= max_a_long; 
       -max_a_lat   <= ((x(2,k+1)-x(2,k))/dt)   <= max_a_lat;    
       -0.3         <=      u(1,k)              <= 0.3;
       -10.0        <=      u(2,k)              <= 5.0;
       
        %         0 <= -1.2*((x(2,k+1)-x(2,k))/dt) + 0.93 - ((x(1,k+1)-x(1,k))/dt);
        %         0 <= 1.2*((x(2,k+1)-x(2,k))/dt) + 0.93 - ((x(1,k+1)-x(1,k))/dt);
        %                 0 >= -1.2*((x(2,k+1)-x(2,k))/dt) - 0.93 - ((x(1,k+1)-x(1,k))/dt);
        %                 0 >= 1.2*((x(2,k+1)-x(2,k))/dt) - 0.93 - ((x(1,k+1)-x(1,k))/dt);
        
       ]; 

end
% objective = objective...
%     + [x(4,k+1); x(1,k+1); x(2,k+1); x(5,k+1)]' *QQ* [x(4,k+1); x(1,k+1); x(2,k+1); x(5,k+1)] ...
%     + LL*[0; x(1,k+1); x(2,k+1); x(5,k+1)] + 200*sc(k+1)^2 + x(3,k+1)*0.03*x(3,k+1);% + 10*sc_V(k+1);

objective = objective + 200*sc(k+1)^2 + 3*x(3,k+1)^2;

constraints = [ constraints;
                left_limit-sc(k+1)  <= x(4,k+1)  	<= right_limit + sc(k+1);  
                0.9                 <= x(1,k+1)   	<= max_Vel_X;
              ]; 
    
    
parameters_in = {x(:,1), curv, pastu, A_1, A_2, A_3, A_4, A_5, A_6, A_7, A_8,...
    A_9, A_10, B_1, B_2, B_3, B_4, left_limit, right_limit};


solutions_out = {u, x(:,2:end)};





%% Fmincon:
% options = sdpsettings('solver','fmincon','verbose',0);

%% Quadprog:
options = sdpsettings('solver','quadprog');

%% Sedumi:
% options = sdpsettings('solver','gurobi','verbose',1);

%% bmibnb:
% options = sdpsettings('solver','bmibnb','verbose',2);



controller = optimizer(constraints, objective,options,parameters_in,solutions_out);


end



