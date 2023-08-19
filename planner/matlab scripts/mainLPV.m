clear

files = "../scripts/data/experiments_paper/LPV3_agent_lh/csv";
addpath(genpath(files))

tools = "tools";
addpath(genpath(tools))

file_tree = dir(files);
dirFlags = [file_tree.isdir];
% Extract only those that are directories.
subFolders = file_tree(dirFlags); % A structure with extra info.
% Get only the folder names into a cell array.
subFolders = {subFolders(3:end).name}; % Start at 3 to skip . and .

for n = 1:length(subFolders)
    
    ag = "/" + subFolders{n};
    u(n,:,:) = import_u(files + ag + "/u.dat");
    time(n,:) = import_t(files + ag + "/time.dat");
    states(n,:,:) = import_s(files + ag + "/states.dat");
    plan_dist(n,:) = import_t(files + ag + "/plan_dist.dat");

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cartesian States 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Agent 0 

total = length(subFolders);
for n = 0:2:(total-1)*2 

    sgtitle("Linear velocity and S evolutions for agents in the fleet")
    subplot(total,2,n+1);
    hold on 
    title("agent " + num2str(n/2 +1))
    plot(states(n/2 +1 ,:,1))
    ylabel("Vel x (m/s)")
    xlabel("it")
    grid on 
    hold off 

    subplot(total,2,n+2);
    hold on 
    title("agent " + num2str(n/2 +1))
    plot(states(n/2 +1,:,7))
    ylabel("s (m)")
    xlabel("it")
    grid on 
    hold off 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Avg Cartesian States 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k = size(time,2); 
min_pl_v = zeros(k,1);
max_pl_v = zeros(k,1);
avg_pl_v = zeros(k,1);
min_pl_s = zeros(k,1);
max_pl_s = zeros(k,1);
avg_pl_s = zeros(k,1);

for it = 1:k 
    min_pl_v(it) = min(states(:,it,1));
    max_pl_v(it) = max(states(:,it,1));
    avg_pl_v(it) = mean(states(:,it,1));
    min_pl_s(it) = min(states(:,it,7));
    max_pl_s(it) = max(states(:,it,7));
    avg_pl_s(it) = mean(states(:,it,7));
end

sgtitle("Linear velocity and S evolutions for agents in the fleet")
subplot(1,2,1);
hold on 
plot(min_pl_v)
plot(max_pl_v)
plot(avg_pl_v)
ylabel("Vel x (m/s)")
xlabel("it")
legend("min", "max", "avg")
grid on 
hold off 

subplot(1,2,2);
hold on 
plot(min_pl_s)
plot(max_pl_s)
plot(avg_pl_s)
ylabel("s (m)")
xlabel("it")
legend("min", "max", "avg")
grid on 
hold off 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % distance  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure()
% sgtitle("distance")
% for n = 1:total
% 
%     sgtitle("Distance between agents along the track")
%     subplot(total,1,n);
%     aux_title = "agent " + num2str(n) + " vs neighbours";
%     title(aux_title)
%     hold on
%     grid on 
%     plot(0.25*ones(size(states,2)))
%     Legend=cell(total,1);
%     Legend{1} = "minimum allowed distance";
%     k = 1;
%     for j = 1:total
%         if j ~= n
%             plot(sqrt((states(n,:,8) - states(j,:,8)).^2 +  (states(n,:,9) - states(j,:,9)).^2))
%             k = k+1;
%             Legend{k} = "agent " + num2str(j);
%             ylabel("distance (m)")
%             xlabel("it")
%         end 
%     end
%     legend(Legend)
%     hold off 
% 
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% avg distance  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k = size(time,2); 
d_placeholder = zeros(total,total,k);
min_placeholder = zeros(k,1);
max_placeholder = zeros(k,1);
avg_placeholder = zeros(k,1);

for j = 1:total
     for n = 1:total
        d_placeholder(j,n,:) = sqrt((states(n,:,8) - states(j,:,8)).^2 +  (states(n,:,9) - states(j,:,9)).^2); 
     end 
end

for it = 1:k
    idx = d_placeholder(:,:,it)>0;
    aux_ds = d_placeholder(:,:,it); 
    min_placeholder(it) = min(aux_ds(idx));
    max_placeholder(it) = max(aux_ds(idx));
    avg_placeholder(it) = mean(aux_ds(idx));
end


figure()
sgtitle("Inter-vehicle distance analisis")
hold on
grid on 
plot(min_placeholder)
plot(max_placeholder)
plot(avg_placeholder)
plot(0.25*ones(size(states,2)))
legend("min", "max", "avg", "limit")
ylabel("distance (m)")
xlabel("it")
hold off 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% look ahead distance  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k = size(time,2); 
min_placeholder = zeros(k,1);
max_placeholder = zeros(k,1);
avg_placeholder = zeros(k,1);

for it = 1:k 
    min_placeholder(it) = min(plan_dist(:,it));
    max_placeholder(it) = max(plan_dist(:,it));
    avg_placeholder(it) = mean(plan_dist(:,it));
end


figure()
sgtitle("Look ahead distance")
hold on
grid on 
plot(min_placeholder)
plot(max_placeholder)
plot(avg_placeholder)
legend("min", "max", "avg")
ylabel("distance (m)")
xlabel("it")
hold off 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computational times  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k = size(time,2); 
d_placeholder = zeros(total,total,k);
min_placeholder = zeros(k,1);
max_placeholder = zeros(k,1);
avg_placeholder = zeros(k,1);

for it = 1:k 
    min_placeholder(it) = min(time(:,it));
    max_placeholder(it) = max(time(:,it));
    avg_placeholder(it) = mean(time(:,it));
end


figure()
sgtitle("Computational time")
hold on
grid on 
plot(min_placeholder)
plot(max_placeholder)
plot(avg_placeholder)
legend("min", "max", "avg")
ylabel("tme (s)")
xlabel("it")
hold off 

