base = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinearControllerObject/dist";
addpath(genpath(base))

ag = "/0"; 
D(:,:,1) = import_s(base + ag + "/states.dat");

ag = "/1"; 
D(:,:,2) = import_s(base + ag + "/states.dat");

ag = "/2"; 
D(:,:,3) = import_s(base + ag + "/states.dat");

ag = "/3"; 
D(:,:,4) = import_s(base + ag + "/states.dat");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Circuit
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("Path followed")
hold on 
xlabel("seconds")
ylabel("it")
plot(0.25*ones(size(D,1)))
Legend=cell(3^2,1); 
l = 1; 
 for R = 1:4
     for C = 1:4
         if R ~= C
            plot(sqrt((D(:,8,R) - D(:,8,C)).^2 +  (D(:,9,R) - D(:,9,C)).^2))
            Legend{l} = string(R) + " - " + string(C); 
            l = l+1; 
         end 
     end
 end

 legend(Legend)

