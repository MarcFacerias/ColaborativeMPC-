files_CS = "../DistributedPlanner/TestsPaperL5";
addpath(genpath(files_CS))
files_LPV = "../NonLinDistribPlanner/TestsPaperL5";
addpath(genpath(files_LPV))

tools = "tools";
addpath(genpath(tools))

files = [files_CS, files_LPV]; 

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
        u(k,n,:,:) = import_u(files + ag + "/u.dat");
        time(k,n,:,:) = import_t(files + ag + "/time.dat");
        states(k,n,:,:) = import_s(files + ag + "/states.dat");
    
    end
end


