function [controller] = Planning_LPV_MPC_Fn_new(Hp, Q, R, L, Ts)

nx = length(Q);
nu = length(R);

x           = sdpvar(nx,1,Hp+1,'full');  % [vx, vy, w, y_e, theta_e]
u           = sdpvar(nu,1,Hp,'full');    % [steer, accel]
delta_u     = sdpvar(nu,1,Hp,'full');
past_u      = sdpvar(nu,1,'full');
left_bound  = sdpvar(1,Hp+1,'full');
right_bound = sdpvar(1,Hp+1,'full');

Ax = sdpvar(1,3,Hp,'full'); Ay = sdpvar(1,3,Hp,'full'); Aw = sdpvar(1,3,Hp,'full');   
Bx = sdpvar(1,2,Hp,'full'); By = sdpvar(1,2,Hp,'full'); Bw = sdpvar(1,2,Hp,'full');   
Cx = sdpvar(1,1,Hp,'full'); Cy = sdpvar(1,1,Hp,'full'); Cw = sdpvar(1,1,Hp,'full');

A11 = sdpvar(1,1,Hp,'full');   B11 = sdpvar(1,1,Hp,'full');
A12 = sdpvar(1,1,Hp,'full');   B12 = sdpvar(1,1,Hp,'full');
A13 = sdpvar(1,1,Hp,'full');   B21 = sdpvar(1,1,Hp,'full'); 
A22 = sdpvar(1,1,Hp,'full');   B31 = sdpvar(1,1,Hp,'full');
A23 = sdpvar(1,1,Hp,'full');   
A32 = sdpvar(1,1,Hp,'full');    
A33 = sdpvar(1,1,Hp,'full');   
A41 = sdpvar(1,1,Hp,'full');   
A42 = sdpvar(1,1,Hp,'full');    
A51 = sdpvar(1,1,Hp,'full');   
A52 = sdpvar(1,1,Hp,'full');    
A45 = sdpvar(1,1,Hp,'full');  

sf  = sdpvar(1,1,Hp+1,'full');

max_a_lat   = 2.5;
max_a_long  = 0.7; 
min_a_long  = -10;
% Hc = 5;

objective   = 0;
constraints = [];

for i = 1:Hp
    
%     if i == 1
%         constraints = [delta_u(:,1,i) == u(:,1,i) - past_u];
%     else
%         constraints = [constraints; delta_u(:,1,i) == u(:,1,i) - u(:,1,i-1)];
%     end  
    
%     if i > Hc % Control Horizont
%         constraints = [constraints; u(1,1,i) == u(1,1,Hc)];
%         constraints = [constraints; u(2,1,i) == u(2,1,Hc)]; 
%     end
    
    objective = objective + x(:,1,i)'*Q*x(:,1,i) + L*x(:,1,i)+ 200*sf(1,1,i)^2 + delta_u(:,1,i)'* R*delta_u(:,1,i);
    constraints = [constraints;
        % Vx
        x(1,1,i+1) == Ax(1,:,i)*x(1:3,1,i) + Bx(1,:,i)*u(:,1,i) + Cx(1,1,i); % ANFIS
        % Vy
        x(2,1,i+1) == Ay(1,:,i)*x(1:3,1,i) + By(1,:,i)*u(:,1,i) + Cy(1,1,i); % ANFIS
        % Wz
        x(3,1,i+1) == x(3,1,i) + (A32(1,1,i)*x(2,1,i) + A33(1,1,i)*x(3,1,i) + B31(1,1,i)*u(1,1,i))*Ts;        
        % y_e
        x(4,1,i+1) == x(4,1,i) + (A41(1,1,i)*x(1,1,i) + A45(1,1,i)*x(5,1,i) + A42(1,1,i)*x(2,1,i)) * Ts;  
        % theta_e
        x(5,1,i+1) == x(5,1,i) + (A51(1,1,i)*x(1,1,i) + A52(1,1,i)*x(2,1,i) + x(3,1,i))*Ts];
        
    constraints = [constraints;
        left_bound-sf(1,1,i) <=  x(4,1,i);
        right_bound+sf(1,1,i) >= x(4,1,i); %(1,:)
                0.85  <=         x(1,1,i)             <=  2.7;      % vx
          min_a_long  <= ((x(1,1,i+1)-x(1,1,i))/Ts)   <= max_a_long; 
         -max_a_lat   <= ((x(2,1,i+1)-x(2,1,i))/Ts)   <= max_a_lat;
               -0.22  <=         u(1,1,i)             <=  0.22;     % steer
               -0.09  <=         u(2,1,i)             <=  1.06      % accel
       ];
end

% Whats sf -> 
objective = objective + x(:,1,i+1)'*Q*x(:,1,i+1) + L*x(:,1,i+1) + 200*sf(1,1,i+1)^2;
constraints = [constraints;
               0.85  <=  x(1,1,i+1)  <=  2.7; % vx
               left_bound-sf(1,1,i) <=  x(4,1,i); % y_e
               right_bound+sf(1,1,i) >= x(4,1,i);
               % left_bound - sf(1,1,i+1)  <=  x(4,1,i+1)  <= right_bound + sf(1,1,i+1); % y_e
              ]; 

inputs  = {x(:,1,1), past_u, B11, B12, B21, B31, A11, A12, A13, A22,...
          A23, A32, A33, A41, A42, A51, A52, A45, Ax, Ay, Aw, Bx, By, Bw, Cx, Cy, Cw, left_bound, right_bound};
% outputs = {u, x(:,1,2:end), objective}; % Solution
outputs = {u, x(:,1,2:end)}; % Solution

%% Quadprog
options = sdpsettings('solver','quadprog');
controller = optimizer(constraints, objective, options, inputs, outputs);

end