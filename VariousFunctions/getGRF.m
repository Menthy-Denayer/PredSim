function GRFres = getGRF(path_GRF,labels_GRF)
% --------------------------------------------------------------------------
% getGRF
%   This function returns the ground reaction forces given the path 
%   to the ground reaction forces file.
%   
% INPUT:
%   - path_GRF -
%   * path to the ground reaction forces file
% 
%   - model_info -
%   * structure with all the model information based on the OpenSim model
%
% OUTPUT:
%   - GRF -
%   * struct with ground reaction forces (time, GRFr, GRFl)
% 
% Original author: Antoine Falisse
% Original date: 12/19/2018
%
% Last edit by: Menthy Denayer
% Last edit date: 10/June/2025 : adapted getIK to extract GRF tracking data
% --------------------------------------------------------------------------

% Load the reference date
GRFall = read_motionFile_v40(path_GRF);

%% Build the struct with inverse kinematics
% First column has the timestamps
GRFres.time = GRFall.data(:,strcmp(GRFall.labels,{'time'}));
GRFres.all(:,1) = GRFres.time;
GRFres.colheaders{1,1} = 'time';

% Add the GRF
for i = 1:length(labels_GRF)
    currGRF = labels_GRF{i};
    idx_grf_GRFall = find(strcmp(GRFall.labels,currGRF),1,'first');
    if isempty(idx_grf_GRFall)
        GRFres.(currGRF) = zeros(size(GRFres.time));
    else
        GRFres.(currGRF) = GRFall.data(:,idx_grf_GRFall);
    end

    GRFres.all(:,i+1) = GRFres.(currGRF);
    GRFres.colheaders{1,i+1} = currGRF;
end

% Low-pass filter
order = 4;
cutoff_low = 6;
fs=1/mean(diff(GRFres.all(:,1)));
[af,bf] = butter(order/2,cutoff_low./(0.5*fs),'low');
GRFres.allfilt = GRFres.all;
GRFres.allfilt(:,2:end) = filtfilt(af,bf,GRFres.allfilt(:,2:end));

end
