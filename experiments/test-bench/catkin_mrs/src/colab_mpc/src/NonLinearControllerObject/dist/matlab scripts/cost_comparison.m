base = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinearControllerObject/dist";
addpath(genpath(base))

c0 = import_cost(base + "/cost_hist.dat");
c1 = import_cost(base + "/cost_hist2.dat");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OCD Time  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("numeric value of the cost")
xlabel("it")
ylabel("cost")
hold on 
plot(c0)
plot(c1)
legend("constrain the agent that is far away","constrain the agent that is close")
hold off 

