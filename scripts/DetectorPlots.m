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


%% Format data
times    = cell(1,numel(detectorTypes));
total_kp = cell(1,numel(detectorTypes));
car_kp   = cell(1,numel(detectorTypes));
ratio = cell(1,numel(detectorTypes));
kp_size = cell(1,numel(detectorTypes));
kp_num = zeros(1,numel(detectorTypes));
size = zeros(1,numel(detectorTypes));
mean_time = zeros(1,numel(detectorTypes));
for i=1:numel(data)
    tmp = data(i);
    name = split(tmp.name(1:end-4),"_");
    detector   = name{1};
    descriptor = name{2};
    
    for k=1:numel(detectorTypes)
        if strcmp(detectorTypes{k}, detector)
            
            times{k} = [times{k}, tmp.t_kp];
            total_kp{k} = [total_kp{k}, tmp.total_kp];
            car_kp{k} = [car_kp{k}, tmp.car_kp];
            for ii=1:9
                kp_size{k} = [kp_size{k}, tmp.kpts{ii}(:,3)'];
            end
            size(k) = numel(times{k});
            kp_num(k) = numel(kp_size{k});
            ratio{k} = [ratio{k}, tmp.a_u./tmp.a_tot];
            mean_time(k) = mean(times{k});
            break;
        end
    end
end
min_size_num = min(kp_num);

[v,idxs] = sort(mean_time);
detectorTypes = detectorTypes(idxs);
times = times(idxs);
total_kp = total_kp(idxs);
car_kp = car_kp(idxs);
size = size(idxs);
mean_time = mean_time(idxs);
ratio = ratio(idxs);
min_n = min(size);
M_times = zeros(min_n, numel(detectorTypes));
M_total_kp = zeros(min_n, numel(detectorTypes));
M_car_kp = zeros(min_n, numel(detectorTypes));
M_ratio = zeros(min_n, numel(detectorTypes));
M_kp_size = zeros(min_size_num, numel(detectorTypes));
for k=1:numel(detectorTypes)
    M_times(:,k) = times{k}(1:min_n);
    M_total_kp(:,k) = total_kp{k}(1:min_n);
    M_car_kp(:,k) = car_kp{k}(1:min_n);
    M_ratio(:,k) = ratio{k}(1:min_n);
    M_kp_size(:,k) = kp_size{k}(1:min_size_num);
end

%% Plot time
fig = figure();
hold on;
box on;
grid on;
title("Keypoints size");
boxplot(M_kp_size,'Notch','off','Labels',detectorTypes)
ylabel("Size [pixel]","Interpreter","tex")
xlabel("Detector","Interpreter","tex")   
set(gca,'FontSize',20)
set(findobj(gca,'type','line'),'linew',2);

fig = figure();
hold on;
box on;
grid on;
title("Keypoints detection time");
boxplot(M_times*1000,'Notch','off','Labels',detectorTypes)
ylabel("Time [ms]","Interpreter","tex")
xlabel("Detector","Interpreter","tex")   
set(gca,'FontSize',20)
set(findobj(gca,'type','line'),'linew',2);

fig = figure();
hold on;
box on;
grid on;
title("Number of detected Keypoints in the image");
boxplot(M_total_kp,'Notch','off','Labels',detectorTypes)
ylabel("Number of Kp []","Interpreter","tex")
xlabel("Detector","Interpreter","tex")   
set(gca,'FontSize',20)
set(findobj(gca,'type','line'),'linew',2);

fig = figure();
hold on;
box on;
grid on;
title("Number of detected Keypoints on car");
boxplot(M_car_kp,'Notch','off','Labels',detectorTypes)
ylabel("Number of Kp []","Interpreter","tex")
xlabel("Detector","Interpreter","tex")   
set(gca,'FontSize',20)
set(findobj(gca,'type','line'),'linew',2);


fig = figure();
hold on;
box on;
grid on;
title("Keypoint area union over total area");
boxplot(M_ratio,'Notch','off','Labels',detectorTypes)
ylabel("Ratio []","Interpreter","tex")
xlabel("Detector","Interpreter","tex")   
set(gca,'FontSize',20)
set(findobj(gca,'type','line'),'linew',2);

%% Compute and plot entropy
step = 10;
min_x = 0;
max_x = 1242;
min_y = 0;
max_y = 375;
edges = {min_x:step:max_x, min_y:step:max_y};
A = step*step;
entropy = zeros(numel(detectorTypes),7);
for i=1:numel(data)
    tmp = data(i);
    name = split(tmp.name(1:end-4),"_");
    detector   = name{1};
    descriptor = name{2};    
    for k=1:numel(detectorTypes)
        if strcmp(detectorTypes{k}, detector)
            for j=1:7
                [N,c] = hist3(tmp.kpts{j}(:,1:2),'Edges',edges);
                totalKp = sum(N(:));
                N_scaled = N/totalKp;
                logic = N_scaled~=0;
                entropy(k,j) = sum(sum(N_scaled(logic) .* log(N_scaled(logic)/A)));
            end
            break;
        end
    end
end

figure();
hold on; 
title('Entropy')
axis tight
box on
grid on
boxplot(entropy','Notch','off','Labels',detectorTypes)
ylabel('\propto entropy', 'interpreter','tex')
set(gca,'FontSize',20)
set(findobj(gca,'type','line'),'linew',2);

    


