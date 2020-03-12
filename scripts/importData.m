%% Init console
% Go to script folder
filename = which(mfilename);
[pathstr,~,ext] = fileparts(filename);
cd(pathstr);

% Clear console and variables
clc;
clear all;
close all;

%% Parameters
dataFolder = "../build/logs";

%% Load file
files = dir(fullfile(dataFolder, '*.txt'));

for idx = 1:numel(files)
    file = files(idx);
    f_path = [file.folder, filesep,file.name];
    display(file.name)
    tmp_struct.name = file.name;
    fid = fopen(f_path,'r');
    
    lane_num = 0;
    while ~feof(fid)
        lane_num = lane_num + 1;
        line = fgetl(fid);
        %if lane_num < 1
        %    continue;
        %end
        splitted = split(line,",");
                
        tmp_struct.camera(lane_num) = str2num(splitted{1});
        tmp_struct.laser(lane_num) = str2num(splitted{2});
    end
    fclose(fid);
    
    if(idx == 1)
        data = tmp_struct;
    else
        data(idx) = tmp_struct;
    end    
end

save("data","data");