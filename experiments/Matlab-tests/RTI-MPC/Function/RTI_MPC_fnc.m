function [ controller ] = RTI_MPC_fnc( Hp, SchedVars_Limits, nx, nu )

x       = sdpvar(nx,1,1,'full');    % [vx, vy, w, ey, etheta]
x_guess = sdpvar(nx,1,Hp,'full');     % [delta, acceleration]
r       = sdpvar(nx,1,Hp,'full');
h       = sdpvar(nx*2+nu*2,1,Hp,'full');
dx      = sdpvar(nx,1,Hp+1,'full');
du      = sdpvar(nu,1,Hp,'full');
A       = sdpvar(nx,nx,Hp,'full'); 
B       = sdpvar(nx,nu,Hp,'full');
C       = sdpvar(nx*2+nu*2,nx,Hp,'full');
D       = sdpvar(nx*2+nu*2,nu,Hp,'full');
%initial guess of the rti 
u_guess = sdpvar(nu,1,Hp,'full');

%references 
rif_x   = sdpvar(nx,1,Hp,'full');
rif_u   = sdpvar(nu,1,Hp,'full');

Qvx     = 1/SchedVars_Limits(1,2)^2;
Qvy     = 1/SchedVars_Limits(2,2)^2;
Qw      = 1/SchedVars_Limits(3,2)^2;
Qdelta  = 1/SchedVars_Limits(4,2)^2;
Qaccel  = 1/SchedVars_Limits(5,2)^2;

Q   = 0.8*diag([ 0.8*Qvx   0.00001*Qvy   0.8*Qw ]);  	% vx, vy, wz, thetaz 
R   = 0.2*diag([ 0.01*Qdelta 0.001*Qaccel ]); %0.5 0.5

H = blkdiag(Q,R);

obj = 0;

for k = 1:Hp
    if k == 1
        constraints = [dx(:,1,1) == x(:,1,1)-x_guess(:,1,1);];  
    end
    
    obj = obj + 1/2*([dx(:,1,k);du(:,1,k)]'*H*[dx(:,1,k);du(:,1,k)])+(H*[x_guess(:,1,k)-rif_x(:,1,k); u_guess(:,1,k)-rif_u(:,1,k)])'*[dx(:,1,k); du(:,1,k)];
    
    constraints = [constraints;
                   dx(:,1,k+1) == A(:,:,k)*dx(:,1,k) + B(:,:,k)*du(:,1,k) + r(:,1,k);
                   C(:,:,k)*dx(:,1,k) + D(:,:,k)*du(:,1,k) + h(:,1,k) <= 0;];
        
end

parameters_in = {x, x_guess, u_guess, rif_x, rif_u, r, h, A, B, C, D};

solutions_out = {du(:,1,1:end), dx(:,1,1:end), obj};

%% Quadprog:
% Why do we use a regular solver instead of a the Newton single iteration ?

options = sdpsettings('solver','quadprog');

controller = optimizer( constraints, obj, options, parameters_in, solutions_out);

end