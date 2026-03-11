function [exo] = kneeExoBryan2020(init, settings_orthosis)
% --------------------------------------------------------------------------
% kneeExoBryan2020
%   Knee exoskeleton that applies a torque profile (torque in function of
%   stride) to the knee. 
%
%   This function requires additional dependencies, which can be downloaded
%   from: 
%   ...
%
%   References
%   [1] Bryan GM, Franks PW, Klein SC, Peuchen RJ, Collins SH. 
%   A hip–knee–ankle exoskeleton emulator for studying gait assistance. 
%   The International Journal of Robotics Research. 2021;40(4-5):722-746. 
%   doi:10.1177/0278364920961452
%
% INPUT:
%   - init -
%   * struct with information used to initialise the Orthosis object.
% 
%   - settings_orthosis -
%   * struct with information about this orthosis, containing the fields:
%       - function_name = kneeExoBryan2020  i.e. name of this function   
%       - dependencies_path path to dependencies
%       - isFullGaitCycle   assistance profile for full stride when true, half stride when false. Default is false.
%       - stiffness_onset:  onset of virtual spring-based assistance
%       - stiffness_offset: offset of virtual spring-based assistance
%       - stiffness:        stiffness of the virtual spring 
%       - peak_torque:      peak torque in Nm
%       - peak_time:        timing of peak as % of stride
%       - rise_time:        rise time as % of stride
%       - fall_time:        fall time as % of stride
%       - damping_onset:    onset of virtual damper-based assistance
%       - damping_offset:   offset of virtual damper-based assistance
%       - damping:          damping of the virtual damper 
%   Values are set via S.orthosis.settings{i} in main.m, with i the index
%   of the orthosis.
%
%
% OUTPUT:
%   - exo -
%   * an object of the class Orthosis
% 
% Original author: Menthy Denayer
% Original date: 10/March/2026
% --------------------------------------------------------------------------

%% Create Orthosis object
exo = Orthosis('exo',init,true);

%% Read settings that were passed from main.m
if isfield(settings_orthosis,'isFullGaitCycle')
    isFullGaitCycle = settings_orthosis.isFullGaitCycle;
else
    isFullGaitCycle = false;
end
% exo_params(1) = settings_orthosis.stiffness_onset;
% exo_params(2) = settings_orthosis.stiffness_offset;
% exo_params(3) = settings_orthosis.stiffness;
% exo_params(4) = settings_orthosis.peak_time;
% exo_params(5) = settings_orthosis.rise_time;
% exo_params(6) = settings_orthosis.fall_time;
% exo_params(7) = settings_orthosis.peak_torque;
% exo_params(8) = settings_orthosis.damping_onset;
% exo_params(9) = settings_orthosis.damping_offset;
% exo_params(10) = settings_orthosis.damping;
side = settings_orthosis.left_right; % 'l' for left or 'r' for right

%% Define number of control intervals for simulation
N_control = exo.getNmesh(); 
% number of control intervals for full stride
if isFullGaitCycle
    N_stride = N_control; 
else
    N_stride = N_control*2;
end

%% Define mesh points for control
mesh_control = (1:N_control);
% if left side, shift mesh by half a stride
if strcmp(side,'l')
    mesh_control = mesh_control + N_stride/2;
    mesh_control = mod(mesh_control-1,N_stride)+1;
end

%% Initialize torque profile
T_knee = zeros(3, N_control);
eps_heavyside = 1;

%% Fill spring assistance
% get joint angles
q_knee = exo.var_coord(['knee_angle_',side]);                               % knee angle in rad

% define torque
w_spring = smooth_window(mesh_control, settings_orthosis.stiffness_onset, settings_orthosis.stiffness_offset, eps_heavyside);
T_spring = -settings_orthosis.stiffness*q_knee.*w_spring';
T_knee(3, :) = T_knee(3, :) + T_spring';

%% Fill damping assistance
% get joint angular velocities
qdot_knee = exo.var_coord(['knee_angle_',side],'vel');                      % knee angular velocity in rad/s

% define torque
w_damp = smooth_window(mesh_control, settings_orthosis.damping_onset, settings_orthosis.damping_offset, eps_heavyside);
T_damp = -settings_orthosis.damping*qdot_knee.*w_damp';
T_knee(3, :) = T_knee(3, :) + T_damp';

%% Fill polynomial assistance
% define spline control points
t_pts = [settings_orthosis.peak_time - settings_orthosis.rise_time, ...
         settings_orthosis.peak_time, ...
         settings_orthosis.peak_time + settings_orthosis.rise_time];

T_pts = [0, settings_orthosis.peak_torque, 0];

% create spline
pp = spline(t_pts, [0 T_pts 0]);

% Evaluate spline only in active region
Tspline = zeros(3,N_control);

idx_spline = (mesh_control >= t_pts(1)) & (mesh_control <= t_pts(3));
Tspline(3, idx_spline) = -ppval(pp, mesh_control(idx_spline));

%% Total knee torque
T_tot = T_knee + Tspline;

%% Apply exo torque on femur and tibia
exo.addBodyMoment(T_tot, ['T_exo_thigh_',side],['femur_',side]);
exo.addBodyMoment(-T_tot, ['T_exo_shank_',side],['tibia_',side],['femur_',side]);


% plot figure if wanted
if isfield(settings_orthosis,'plotAssistanceProfile')
    if isa(settings_orthosis.plotAssistanceProfile,'matlab.ui.Figure')
        figure(settings_orthosis.plotAssistanceProfile)
        plotAssistanceProfile = true;
    elseif settings_orthosis.plotAssistanceProfile
        figure();
        plotAssistanceProfile = true;
    else
        plotAssistanceProfile = false;
    end

    if plotAssistanceProfile
        if strcmp(side,'l')
            legName = 'left';
        else
            legName = 'right';
        end
        hold on
        plot(mesh_control/N_stride*100,-T_tot(3,:),'DisplayName',legName)
        xlabel('Stride [%]')
        ylabel(["Assistance [Nm]";"<-- Extension Flexion -->"])
        title('kneeExoBryan2020')
        legend('Location','best')

    end

end

end

function w = smooth_window(t, t_on, t_off, eps)

    S_on  = 0.5*(1 + tanh((t - t_on)/eps));
    S_off = 0.5*(1 - tanh((t - t_off)/eps));
    
    w = S_on .* S_off;

end