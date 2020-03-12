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

%% Plots time vs matches
fig1 = figure();
hold on; grid on; axis tight; box on;
set(gca,'FontSize',20)
xlabel("computation time [ms]",'Interpreter','tex');
ylabel("matches number []",'Interpreter','tex');
times    = cell(1,numel(detectorTypes));
total_kp = cell(1,numel(detectorTypes));
car_kp   = cell(1,numel(detectorTypes));
ratio = cell(1,numel(detectorTypes));
size = zeros(1,numel(detectorTypes));
mean_time = zeros(1,numel(detectorTypes));
h = zeros(numel(detectorTypes)+numel(descriptorTypes)+5, 1);
h(1) = plot(NaN,NaN,'w','DisplayName','DETECTORS');
h(2) = plot(NaN,NaN,'w','DisplayName','');
h(numel(detectorTypes)+3) = plot(NaN,NaN,'w','DisplayName','');
h(numel(detectorTypes)+4) = plot(NaN,NaN,'w','DisplayName','DESCRIPTORS');
h(numel(detectorTypes)+5) = plot(NaN,NaN,'w','DisplayName','');
for k=1:numel(detectorTypes)
  h(k+2) = plot(NaN,NaN,['k',markerType{k}],'DisplayName',detectorTypes{k},'linewidth', 2);
end
for k=1:numel(descriptorTypes)
  h(k+numel(detectorTypes)+5) = plot(NaN,NaN,colors{k},'DisplayName',descriptorTypes{k},'linewidth', 2);
end

for i=1:numel(data)
    tmp = data(i);
    name = split(tmp.name(1:end-4),"_");
    detector   = name{1};
    descriptor = name{2};
    
    for k=1:numel(detectorTypes)
        if strcmp(detectorTypes{k}, detector)
                for j=1:numel(detectorTypes)
                    if strcmp(descriptorTypes{j}, descriptor)
                        xs = i*ones([1,numel(tmp.camera)]);                        
                        plot(xs,tmp.camera - tmp.laser,[markerType{k},colors{j}],'MarkerSize', 18, 'DisplayName', tmp.name(1:end-4),'linewidth',2);
                        break;
                    end 
                end

            break;
        end
    end
end
legend(h,'orientation','horizontal','NumColumns',2);

