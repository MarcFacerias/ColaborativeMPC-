function h = H_model(SchedVars_Limits,x_guess,u_guess,nx,nu)

h = zeros(nx*2+nu*2,1);

h(1) = SchedVars_Limits(1,1) - x_guess(1,1);
h(2) = x_guess(1,1) - SchedVars_Limits(1,2);
h(3) = SchedVars_Limits(2,1) - x_guess(2,1);
h(4) = x_guess(2,1) - SchedVars_Limits(2,2);
h(5) = SchedVars_Limits(3,1) - x_guess(3,1);
h(6) = x_guess(3,1) - SchedVars_Limits(3,2);
h(7) = SchedVars_Limits(4,1) - u_guess(1,1);
h(8) = u_guess(1,1) - SchedVars_Limits(4,2);
h(9) = SchedVars_Limits(5,1) - u_guess(2,1);
h(10) = u_guess(2,1) - SchedVars_Limits(5,2);

end