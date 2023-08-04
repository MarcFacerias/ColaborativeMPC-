clear
files_CS = "../data/NL3_agent_sh/csv";
addpath(genpath(files_CS))
files_LPV = "../data/LPV3_agent/csv";
addpath(genpath(files_LPV))

tools = "tools";
addpath(genpath(tools))

files = [files_LPV, files_CS ]; 
ts_LPV = 0.025;
ts_NL = 0.01;

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
        time_aux(n,:,:) = import_t(files(k) + ag + "/time.dat");
        states_aux(n,:,:) = import_s(files(k) + ag + "/states.dat");
    
    end
    
    u{k} = u_aux;
    time{k} = time_aux;
    states{k} = states_aux;

    clear u_aux time_aux states_aux
end

time_y{1} = ts_LPV * (1:size(states{1},2));
time_y{2} = ts_NL  * (1:size(states{2},2));

total = length(subFolders);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cartesian States 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Agent 0 

% for n = 0:2:(total-1)*2 
% 
%     sgtitle("Linear velocity and S evolutions for agents in the fleet")
%     subplot(total,2,n+1);
%     hold on 
%     title("agent " + num2str(n/2 +1))
%     Legend=cell(2,1);
%     k = 1;
%     for i =1:2
%         plot(time_y{i}, states{i}(n/2 +1 ,:,1))
%         Legend{k} = alg{i};
%     end
%     legend(Legend)
%     ylabel("Vel x (m/s)")
%     xlabel("s")
%     grid on 
%     hold off 
% 
%     subplot(total,2,n+2);
%     hold on 
%     title("agent " + num2str(n/2 +1))
%     for i =1:2
%         plot(time_y{i}, states{i}(n/2 +1,:,7))
%     end
%     ylabel("s (m)")
%     xlabel("s")
%     grid on 
%     hold off 
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distance  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure()
sgtitle("distance")
for n = 0:2:(total-1)*2  
    for i = 1:2
    
        sgtitle("Distance between agents along the track")
        subplot(total,2,n+i);
        aux_title = "agent " + num2str(n/2+1) + " vs neighbours";
        title(aux_title)
        hold on
        grid on 
        plot(0.25*ones(size(states{i},2)))
        Legend=cell(total*2-3,1);
        Legend{1} = "minimum allowed distance";
        k = 1;
        for j = 1:total
            if j ~= n/2 + 1
                plot(sqrt((states{i}(n/2+1,:,8) -  states{i}(j,:,8)).^2 +  ( states{i}(n/2+1,:,9) -  states{i}(j,:,9)).^2))
                k = k+1;
                Legend{k} = "agent " + num2str(j);
                ylabel("distance (m)")
                xlabel("it")
            end 
        end
        legend(Legend)
        hold off 
    end
end


