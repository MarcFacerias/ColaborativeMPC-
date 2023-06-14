base = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinearControllerObject/dist";
addpath(genpath(base))
ag = "/0"; 

u_0 = import_u(base + ag + "/u.dat");
time_0 = import_t(base + ag + "/time.dat");
states_0 = import_s(base + ag + "/states.dat");

ag = "/1"; 

u_1 = import_u(base + ag + "/u.dat");
time_1 = import_t(base + ag + "/time.dat");
states_1 = import_s(base + ag + "/states.dat"); 
ch/catkin_mrs/src/colab_mpc/src/NonLinearControllerObject/dist";
time_OCD = import_ocd(base + "/time_OCD.dat");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cartesian States 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Agent 0 
hold on 
sgtitle("Cartesian states")
subplot(6,2,1);
plot(states_0(:,1))
ylabel("Vel X")
xlabel("it")

subplot(6,2,3);
plot(states_0(:,2))
ylabel("Vel Y")
xlabel("it")

subplot(6,2,5);
plot(states_0(:,3))
ylabel("w")
xlabel("it")

subplot(6,2,7);
plot(states_0(:,6))
ylabel("theta")
xlabel("it")

subplot(6,2,9);
plot(states_0(:,8))
ylabel("x")
xlabel("it")

subplot(6,2,11);
plot(states_0(:,9))
ylabel("y")
xlabel("it")

% Agent 1 
subplot(6,2,2);
plot(states_1(:,1))
ylabel("Vel X")
xlabel("it")

subplot(6,2,4);
plot(states_1(:,2))
ylabel("Vel Y")
xlabel("it")

subplot(6,2,6);
plot(states_1(:,3))
ylabel("w")
xlabel("it")

subplot(6,2,8);
plot(states_1(:,6))
ylabel("theta")
xlabel("it")

subplot(6,2,10);
plot(states_1(:,8))
ylabel("x")
xlabel("it")

subplot(6,2,12);
plot(states_1(:,9))
ylabel("y")
xlabel("it")
hold off 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frenet States 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Agent 0 
hold on 
sgtitle("Frenet states")
subplot(3,2,1);
plot(states_0(:,7))
ylabel("s")
xlabel("it")

subplot(3,2,3);
plot(states_0(:,4))
ylabel("ey")
xlabel("it")

subplot(3,2,5);
plot(states_0(:,5))
ylabel("eth")
xlabel("it")

% Agent 1 
subplot(3,2,2);
plot(states_1(:,1))
ylabel("Vel X")
xlabel("it")

subplot(3,2,4);
plot(states_1(:,2))
ylabel("Vel Y")
xlabel("it")

subplot(3,2,6);
plot(states_1(:,3))
ylabel("w")
xlabel("it")
hold off 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OCD Time  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("OCD time")
xlabel("seconds")
ylabel("it")
plot(time_OCD)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Circuit
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("Path followed")
hold on 
xlabel("seconds")
ylabel("it")
plot(states_0(:,8),states_0(:,9), ".r")
plot(states_1(:,8),states_1(:,9), ".g")