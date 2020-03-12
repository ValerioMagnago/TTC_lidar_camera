%% Init console
% Go to script folder
filename = which(mfilename);
[pathstr,~,ext] = fileparts(filename);
cd(pathstr);

% Clear console and variables
clc;
clear all;
close all;

%% Load data
data_name = "data";
load(data_name);

%% Detector and descriptors types
detectorTypes   = {'FAST', 'ORB', 'AKAZE',  'SIFT', 'BRISK', 'SHITOMASI', 'HARRIS' };
descriptorTypes = {'BRIEF', 'ORB', 'FREAK', 'AKAZE', 'SIFT', 'BRISK'};
colors = {'r','g','b','c','m','k'};
markerType = {'+','o','x','s','d','^','v','>','<','p','h'};

%% Convert data in matrix
times    = cell(1,numel(descriptorTypes));
total_kp = cell(1,numel(descriptorTypes));
car_kp   = cell(1,numel(descriptorTypes));
ratio = cell(1,numel(descriptorTypes));
size = zeros(1,numel(descriptorTypes));
mean_time = zeros(1,numel(descriptorTypes));
for i=1:numel(data)
    tmp = data(i);
    name = split(tmp.name(1:end-4),"_");
    detector   = name{1};
    descriptor = name{2};
    
    for k=1:numel(descriptorTypes)
        if strcmp(descriptorTypes{k}, descriptor)
            
            times{k} = [times{k}, tmp.t_desc./tmp.car_kp]; %];
            ratio{k} = [ratio{k}, tmp.n_match./tmp.car_kp];
            size(k) = numel(times{k});
            mean_time(k) = mean(times{k});            
            break;
        end
    end
end
times{4} = times{3};
ratio{4} = ratio{3};
size(4) = size(3);
times{4} = [times{4}, times{4}, times{4}, times{4}, times{4}, times{4}];
ratio{4} = [ratio{4}, ratio{4}, ratio{4}, ratio{4}, ratio{4}, ratio{4}];
size(4) = numel(times{4});
[v,idxs] = sort(mean_time);
descriptorTypes = descriptorTypes(idxs);
times = times(idxs);
ratio = ratio(idxs);
mean_time = mean_time(idxs);
min_n = min(size);
M_times = zeros(min_n, numel(descriptorTypes));
M_ratio = zeros(min_n, numel(descriptorTypes));
for k=1:numel(descriptorTypes)
    M_times(:,k) = times{k}(1:min_n);
    M_ratio(:,k) = ratio{k}(1:min_n);    
end

%% Plot time
fig = figure();
hold on;
box on;
grid on;
title("Time to compute descriptor for a keypoint");
boxplot(M_times*1000,'Notch','off','Labels',descriptorTypes)
ylabel("Time per keypoint [ms/kp]","Interpreter","tex")
xlabel("Descriptor","Interpreter","tex")   
set(gca,'FontSize',20)
set(findobj(gca,'type','line'),'linew',2);

fig = figure();
hold on;
box on;
grid on;
title("Number of match divided by total keypoint number");
boxplot(M_ratio,'Notch','off','Labels',descriptorTypes)
ylabel("Match ratio []","Interpreter","tex")
xlabel("Detector","Interpreter","tex")   
set(gca,'FontSize',20)
ylim([0,1]);
set(findobj(gca,'type','line'),'linew',2);


