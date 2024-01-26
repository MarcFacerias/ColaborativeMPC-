function [x_guess, u_guess] = Shift_Procedure(x,u,Hp,nx,nu,Tc)
    
x_guess = zeros(nx,1,Hp+1);
u_guess = zeros(nu,1,Hp);

for k = 1:Hp
    x_guess(:,1,k) = x(:,1,k);
    u_guess(:,1,k) = u(:,1,k); 
end

x_guess(:,1,Hp+1) = F_model(x(:,1,Hp),u(:,1,Hp),Tc);

end