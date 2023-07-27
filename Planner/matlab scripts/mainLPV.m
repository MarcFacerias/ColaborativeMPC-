files = "../DistributedPlanner/TestsPaperL5";
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
hold on 
total = length(subFolders);
for n = 0:2:(total-1)*2 

    sgtitle("Linear velocity and S evolutions for agents 1 to N")
    subplot(total,2,n+1);
    plot(states(n/2 +1 ,:,1))
    ylabel("Vel X")
    xlabel("it")
    
    subplot(total,2,n+2);
    plot(states(n/2 +1,:,7))
    ylabel("s")
    xlabel("it")

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distance  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("distance")
for n = 1:total

    sgtitle("Distance between agents along the track")
    subplot(total,1,n);
    hold on
    plot(0.3*ones(size(states,2)), "-r")
    for j = 1:total
        if j ~= n
            plot(sqrt((states(n,:,8) - states(j,:,8)).^2 +  (states(n,:,9) - states(j,:,9)).^2))
        end 
    end
    hold off 
  
end
