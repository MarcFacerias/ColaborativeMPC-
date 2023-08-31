clear
files_CS = "../scripts/experiments_paper/NL_3agents_def/csv";
addpath(genpath(files_CS))
files_LPV = "../scripts/experiments_paper/LPV3r_agent_laptop/csv";
addpath(genpath(files_LPV))

tools = "tools";
addpath(genpath(tools))

files = [files_LPV, files_CS ]; 
ts_LPV = 0.025;
ts_NL = 0.025;
ts = [ts_LPV,ts_NL];
alg{1} = "LPV DMPC";
alg{2} = "NL DMPC";

for i = 1:2
    file_tree = dir(files(i));
    dirFlags = [file_tree.isdir];
    % Extract only those that are directories.
    aux = file_tree(dirFlags); % A structure with extra info.
    % Get only the folder names into a cell array.
    subFolders(i,:) = {aux(3:end).name}; % Start at 3 to skip . and 

end

for k = 1 : 2
    for n = 1:length(subFolders(k,:))
        
        ag = "/" + subFolders{k,n};
        u_aux(n,:,:) = import_u(files(k) + ag + "/u.dat");
        time_aux(n,:) = import_t(files(k) + ag + "/time.dat");
        states_aux(n,:,:) = import_s(files(k) + ag + "/states.dat");
        plan_dist_aux(n,:) = import_t(files(k) + ag + "/plan_dist.dat");
        
    end
    
    u{k} = u_aux;
    time{k} = time_aux;
    states{k} = states_aux;
    plan_dist{k} = plan_dist_aux;
    clear u_aux time_aux states_aux plan_dist_aux
end

time_y{1} = ts_LPV * (1:size(states{1},2));
time_y{2} = ts_NL  * (1:size(states{2},2));

total = length(subFolders);
% % 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Cartesian States 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 
% % for n = 0:2:(total-1)*2 
% % 
% %     sgtitle("Linear velocity and S evolutions for agents in the fleet")
% %     subplot(total,2,n+1);
% %     hold on 
% %     title("agent " + num2str(n/2 +1))
% %     Legend=cell(2,1);
% %     for i =1:2
% %         plot(time_y{i}, states{i}(n/2 +1 ,:,1))
% %         Legend{i} = alg{i};
% %     end
% %     legend(Legend)
% %     ylabel("Vel x (m/s)")
% %     xlabel("s")
% %     grid on 
% %     hold off 
% % 
% %     subplot(total,2,n+2);
% %     hold on 
% %     title("agent " + num2str(n/2 +1))
% %     for i =1:2
% %         plot(time_y{i}, states{i}(n/2 +1,:,7))
% %     end
% %     legend(Legend)
% %     ylabel("s (m)")
% %     xlabel("s")
% %     grid on 
% %     hold off 
% % 
% % end

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % distance  
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 
% % figure()
% % sgtitle("distance")
% % for n = 0:2:(total-1)*2  
% %     for i = 1:2
% %     
% %         sgtitle("Distance between agents along the track")
% %         subplot(total,2,n+i);
% %         aux_title = "agent " + num2str(n/2+1) + " vs neighbours";
% %         title(aux_title)
% %         hold on
% %         grid on 
% %         plot(0.25*ones(size(states{i},2)))
% %         Legend=cell(total*2-3,1);
% %         Legend{1} = "minimum allowed distance";
% %         k = 1;
% %         for j = 1:total
% %             if j ~= n/2 + 1
% %                 plot(sqrt((states{i}(n/2+1,:,8) -  states{i}(j,:,8)).^2 +  ( states{i}(n/2+1,:,9) -  states{i}(j,:,9)).^2))
% %                 k = k+1;
% %                 Legend{k} = "agent " + num2str(j);
% %                 ylabel("distance (m)")
% %                 xlabel("it")
% %             end 
% %         end
% %         legend(Legend)
% %         hold off 
% %     end
% % end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Avg Cartesian States 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for as = 1:2 
    k = size(time{as},2); 
    min_pl_v{as} = zeros(k,1);
    max_pl_v{as} = zeros(k,1);
    avg_pl_v{as} = zeros(k,1);
    min_pl_s{as} = zeros(k,1);
    max_pl_s{as} = zeros(k,1);
    avg_pl_s{as} = zeros(k,1);
    for it = 1:k 
        min_pl_v{as}(it) = min(states{as}(:,it,1));
        max_pl_v{as}(it) = max(states{as}(:,it,1));
        avg_pl_v{as}(it) = mean(states{as}(:,it,1));
        min_pl_s{as}(it) = min(states{as}(:,it,7));
        max_pl_s{as}(it) = max(states{as}(:,it,7));
        avg_pl_s{as}(it) = mean(states{as}(:,it,7));
    end
end

figure();
% sgtitle("Linear velocity evolution of agents in the fleet")
hold on
for as = 1:2 
%     plot(time_y{as},min_pl_v{as})
%     plot(time_y{as},max_pl_v{as})
    plot(time_y{as},avg_pl_v{as}, '.-')
end
ylabel("Vel x (m/s)")
xlabel("time (s)")
legend("LPV-MPC", "NL-DMPC")
grid on 
hold off 
saveas(gcf, "figs/vel.eps", "epsc")
savefig("figs/vel")

figure();
% sgtitle("S evolution of agents in the fleet")
hold on 
for as = 1:2 
%     plot(time_y{as},min_pl_s)
%     plot(time_y{as},max_pl_s)
    plot(time_y{as},avg_pl_s{as}, '.-')
end
ylabel("s (m)")
xlabel("time (s)")
grid on 
hold off 
legend("LPV-MPC", "NL-DMPC")
saveas(gcf, "figs/s.eps", "epsc")
savefig("figs/s")
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Avg Distance 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for as = 1:2 
    k = size(time{as},2); 
    min_ivd{as} = zeros(k,1);
    max_ivd{as} = zeros(k,1);
    avg_ivd{as} = zeros(k,1);
    d_placeholder{as} = zeros(total,total,k);

    for j = 1:total
         for n = 1:total
            d_placeholder{as}(j,n,:) = sqrt((states{as}(n,:,8) - states{as}(j,:,8)).^2 +  (states{as}(n,:,9) - states{as}(j,:,9)).^2); 
         end 
    end

    for it = 1:k
        idx = d_placeholder{as}(:,:,it)>0;
        aux_ds = d_placeholder{as}(:,:,it); 
        min_ivd{as}(it) = min(aux_ds(idx));
        max_ivd{as}(it) = max(aux_ds(idx));
        avg_ivd{as}(it) = mean(aux_ds(idx));
    end
end

figure()
% sgtitle("Inter-vehicle distance avg distance")
hold on
for as = 1:2 
    plot(time_y{as},avg_ivd{as}, '.-')
end
ylabel("distance (m)")
xlabel("time (s)")
grid on 
hold off 
legend("LPV-MPC  avg","NL-DMPC avg")
saveas(gcf, "figs/d_avg.eps", "epsc")
savefig("figs/d_avg")

figure()
% sgtitle("Inter-vehicle max distance")
hold on
for as = 1:2 
    plot(time_y{as},max_ivd{as}, '.-')
end
ylabel("distance (m)")
xlabel("time (s)")
grid on 
hold off 
legend("LPV-MPC  max", "NL-DMPC  max")
saveas(gcf, "figs/d_ma.eps", "epsc")
savefig("figs/d_max")

figure()
% sgtitle("Inter-vehicle min distance")
hold on
for as = 1:2 
    plot(time_y{as},min_ivd{as}, '.-')
end
ylabel("distance (m)")
xlabel("time (s)")
grid on 
hold off 
legend("LPV-MPC min", "LPV-MPC  max", "LPV-MPC  avg","NL-DMPC min", "NL-DMPC  max", "NL-DMPC avg" )
saveas(gcf, "figs/d_min.eps", "epsc")
savefig("figs/d_min")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% look ahead distance  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for as = 1:2 
    k = size(time{as},2); 
    min_la{as} = zeros(k,1);
    max_la{as} = zeros(k,1);
    avg_la{as} = zeros(k,1);

    for it = 1:k 
        min_la{as}(it) = min(plan_dist{as}(:,it));
        max_la{as}(it) = max(plan_dist{as}(:,it));
        avg_la{as}(it) = mean(plan_dist{as}(:,it));
    end
end

figure()
% sgtitle("Look ahead distance")
hold on
for as = 1:2 
%     plot(time_y{as},min_la{as})
%     plot(time_y{as},max_la{as})
    plot(time_y{as},avg_la{as}, '.-')
end
ylabel("distance (m)")
xlabel("time (s)")
grid on 
hold off
legend("LPV-MPC", "NL-DMPC")
saveas(gcf, "figs/la_avg.eps", "epsc")
savefig("figs/la_avg")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Computational time  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for as = 1:2 
    k = size(time{as},2); 
    min_t{as} = zeros(k,1);
    max_t{as} = zeros(k,1);
    avg_t{as} = zeros(k,1);

    for it = 1:k 
        min_t{as}(it) = min(time{as}(:,it));
        max_t{as}(it) = max(time{as}(:,it));
        avg_t{as}(it) = mean(time{as}(:,it));
    end
end

figure()
% sgtitle("Computational time of the agorithms")
hold on
for as = 1:2 
%     plot(time_y{as},min_la{as})
%     plot(time_y{as},max_la{as})
    plot(time_y{as},avg_t{as}, '.-')
end
ylabel("comp. time (s)")
xlabel("time (s)")
grid on 
hold off
legend("LPV-MPC", "NL-DMPC")
saveas(gcf, "figs/t_avg.eps", "epsc")
savefig("figs/t_avg")

figure()
% sgtitle("Computational time of the agorithms zoom in")
hold on
for as = 1:2 
%     plot(time_y{as},min_la{as})
%     plot(time_y{as},max_la{as})
    idx = (8/ts(as)):((14/ts(as)));
    plot(time_y{as}(idx),avg_t{as}(idx), '.-')
end
ylabel("comp. time (s)")
xlabel("time (s)")
grid on 
hold off
legend("LPV-MPC", "NL-DMPC")
saveas(gcf, "figs/t_avg_zn.eps", "epsc")
savefig("figs/t_avg_zn")