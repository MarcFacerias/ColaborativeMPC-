files = "../NonLinDistribPlanner/TestsPaperL5";
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
% distance  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("distance")
for n = 1:total

    sgtitle("Distance between agents along the track")
    subplot(total,1,n);
    aux_title = "agent " + num2str(n) + " vs neighbours";
    title(aux_title)
    hold on
    grid on 
    plot(0.25*ones(size(states,2)))
    Legend=cell(total,1);
    Legend{1} = "minimum allowed distance";
    k = 1;
    for j = 1:total
        if j ~= n
            plot(sqrt((states(n,:,8) - states(j,:,8)).^2 +  (states(n,:,9) - states(j,:,9)).^2))
            k = k+1;
            Legend{k} = "agent " + num2str(j);
            ylabel("distance (m)")
            xlabel("it")
        end 
    end
    legend(Legend)
    hold off 
  
end
