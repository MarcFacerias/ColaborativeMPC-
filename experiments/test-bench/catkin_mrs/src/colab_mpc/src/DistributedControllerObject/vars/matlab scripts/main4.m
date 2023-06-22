base = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/DistributedControllerObject/vars";
addpath(genpath(base))
ag = "/0"; 

u_0 = import_u(base + ag + "/u.dat");
time_0 = import_t(base + ag + "/time.dat");
states_0 = import_s(base + ag + "/states.dat");

ag = "/1"; 

u_1 = import_u(base + ag + "/u.dat");
time_1 = import_t(base + ag + "/time.dat");
states_1 = import_s(base + ag + "/states.dat"); 

ag = "/2"; 

u_2 = import_u(base + ag + "/u.dat");
time_2 = import_t(base + ag + "/time.dat");
states_2 = import_s(base + ag + "/states.dat");

ag = "/3"; 

u_3 = import_u(base + ag + "/u.dat");
time_3 = import_t(base + ag + "/time.dat");
states_3 = import_s(base + ag + "/states.dat"); 

states(:,:,1) = states_0(1:550,:); 
states(:,:,2) = states_1(1:550,:); 
states(:,:,3) = states_2(1:550,:);  
states(:,:,4) = states_3(1:550,:);  

u(:,:,1) = u_0(1:550,:);  
u(:,:,2) = u_1(1:550,:);  
u(:,:,3) = u_2(1:550,:);  
u(:,:,4) = u_3(1:550,:);  

time(:,:,1) = time_0(1:550,:);  
time(:,:,2) = time_1(1:550,:);  
time(:,:,3) = time_2(1:550,:); 
time(:,:,4) = time_3(1:550,:); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cartesian States 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Agent 0 
 
%     sgtitle("Cartesian states")
%     subplot(2,3,1);
%     hold on 
%     for i = 1:4
%         plot(states(:,1,i))
%     end
%     ylabel("Vel X")
%     xlabel("it")
%     
%     subplot(2,3,2);
%     hold on 
%     for i = 1:4
%         plot(states(:,2,i))
%     end
%     ylabel("Vel Y")
%     xlabel("it")
%     
%     subplot(2,3,3);
%     hold on 
%     for i = 1:4
%      plot(states(:,3,i))
%     end
%     ylabel("w")
%     xlabel("it")
%     
%     subplot(2,3,4);
%     hold on 
%     for i = 1:4
%         plot(states(:,6,i))
%     end
%     ylabel("theta")
%     xlabel("it")
%     
%     subplot(2,3,5);
%     hold on 
%     for i = 1:4
%         plot(states(:,8,i))
%     end
%     ylabel("x")
%     xlabel("it")
%     
%     subplot(2,3,6);
%     hold on 
%     for i = 1:4
%         plot(states(:,9,i))
%     end
%     ylabel("y")
%     xlabel("it")
%    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Time  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("OCD time")
xlabel("seconds")
ylabel("it")
hold on 
    plot(time_0)
    plot(time_1)
    plot(time_2)
    plot(time_3)
hold off 
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
plot(states_2(:,8),states_2(:,9), ".y")
plot(states_3(:,8),states_3(:,9), ".c")