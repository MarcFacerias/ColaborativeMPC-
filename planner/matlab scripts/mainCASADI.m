clear
close all 

%files = "../scripts/experiments_paper/NL_3agents_lh_erratic/csv";
files = "../scripts/data/experiments_paper/NL_3agents_def/csv";
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
    time(n,:,:) = import_t(files + ag + "/time.dat");
    states(n,:,:) = import_s(files + ag + "/states.dat");

end

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % States 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Agent 0 
% % 
% % total = length(subFolders);
% % 
% % for n = 1 :total
% %     figure('visible','off')
% %     sgtitle("Linear velocity and S evolutions for Agents in the fleet")
% %     for j = 1:9 
% %         subplot(3,3,j);
% %         hold on 
% %         title("state: " + string(j))
% %         plot(states(n,:,j))
% %         ylabel("Value")
% %         xlabel("it")
% %         grid on 
% %         hold off 
% %     end
% %     
% %     figure('visible','off')
% %     for i=1:2
% %         subplot(2,1,i);
% %         plot(u(n,:,i))
% %     end
% % end
% % exportgraphics(gcf, "figsECC/v.eps", "epsc")
% % savefig("figsECC/v")
% % 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % Velocity
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 
% figure('visible','off')
total = length(subFolders);
% for n = 0:2:(total-1)*2 
% for n = 1:total
% 
% %     sgtitle("Linear velocity and S evolutions for Agents in the fleet")
%     subplot(total,1,n);
%     hold on 
%     title("Agent " + num2str(n))
%     plot(states(n ,:,1))
%     ylabel("Vel x (m/s)")
%     xlabel("it")
%     grid on 
%     hold off 
% 
% end
% exportgraphics(gcf, "figsECC/Cstates.pdf")
% savefig("figsECC/Cstates")
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % distance  
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 
figure('visible','on')
% sgtitle("distance")
tiledlayout(3,1,"TileSpacing","compact");
for n = 1:total

    % sgtitle("Distance between Agents along the track")
    % subplot(total,1,n);
    nexttile
    aux_title = "Agent " + num2str(n) + " vs neighbours";
    title(aux_title)
    hold on
    grid on 
    ylim([0 1])
    plot(0.25*ones(size(states,2)))
    Legend=cell(total,1);
    Legend{1} = "min. dist.";
    k = 1;
    for j = 1:total
        if j ~= n
            plot(sqrt((states(n,:,8) - states(j,:,8)).^2 +  (states(n,:,9) - states(j,:,9)).^2))
            k = k+1;
            Legend{k} = "Agent " + num2str(j);
            ylabel("distance (m)")
            xlabel("it")
        end 
    end
    legend(Legend)
    hold off 
    % annotation('rectangle',[0 0 1 1],'Color','w');
end

exportgraphics(gcf, "figsECC/dist.eps")
savefig("figsECC/dist")
% 
% figure('visible','off')
% sgtitle("Computational time of the agorithms")
% hold on
% plot(max(time,1), '.-')
% plot(mean(time,1), '.-')
% plot(min(time,1), '.-')
% 
% ylabel("comp. time (s)")
% xlabel("time (s)")
% grid on 
% hold off
% legend("mean")
% exportgraphics(gcf, "figsECC/time.pdf")
% savefig("figsECC/t_avg")
% 
% figure('visible','off')
% sgtitle("Computational time of the agorithms zoom in")
% hold on
% 
% idx = (8/0.025):((14/0.025));
% plot(max(time(:,idx),1), '.-')
% plot(mean(time(:,idx),1), '.-')
% plot(min(time(:,idx),1), '.-')
% ylabel("comp. time (s)")
% xlabel("time (s)")
% grid on 
% hold off
% legend("mean")
% exportgraphics(gcf, "figsECC/t_avg_zn.pdf")
% savefig("figsECC/t_avg_zn")
% 
% close all 