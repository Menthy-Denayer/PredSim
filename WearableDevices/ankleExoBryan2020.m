function [exo] = ankleExoBryan2020(init, settings_orthosis)
% --------------------------------------------------------------------------
% ankleExoBryan2020
%   Ankle exoskeleton that applies a torque profile (torque in function of
%   stride) to the ankle. 
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
%       - function_name = ankleExoBryan2020  i.e. name of this function   
%       - dependencies_path path to dependencies
%       - isFullGaitCycle:      assistance profile for full stride when true, half stride when false. Default is false.
%       - rise_time:            rise time of the torque, as % of stride
%       - peak_time:            peak time of the torque, as % of stride
%       - fall_time:            fall time of the torque, as % of stride
%       - peak_torque:          peak torque in Nm/kg
%       - upper_body:           name of the upper body to apply torque on
%       - lower_body:           name of the lower body to apply torque on
%       - bodymass:             mass of the body to scale stiffness and peak torque
%
%   Values are set via S.orthosis.settings{i} in main.m, with i the index
%   of the orthosis.
%
%
% OUTPUT:
%   - exo -
%   * an object of the class Orthosis
% 
% Original author: Menthy Denayer
% Original date: 15/July/2026
% --------------------------------------------------------------------------

%% Create Orthosis object
exo = Orthosis('exo',init,true);

%% Read settings that were passed from main.m
if isfield(settings_orthosis,'isFullGaitCycle')
    isFullGaitCycle = settings_orthosis.isFullGaitCycle;
else
    isFullGaitCycle = false;
end

%% Define Variables
upper_body_name = settings_orthosis.upper_body;
lower_body_name = settings_orthosis.lower_body;
side = settings_orthosis.left_right;                                        % 'l' for left or 'r' for right

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

%% Unscale Variables
peak_torque = settings_orthosis.peak_torque * settings_orthosis.bodymass;

%% Define number of control intervals for simulation
N_control = 100; 
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

%% Check Peak Time
% Check if torque is applied beyond 65% of the gait cycle, otherwise adapt 
% the fall time

if(settings_orthosis.peak_time + settings_orthosis.fall_time > 0.65)
    settings_orthosis.fall_time = 0.65 - settings_orthosis.peak_time;
end

%% Fill polynomial extension assistance
Tspline = zeros(3, N_control);

% define spline control points
t_pts = [settings_orthosis.peak_time - settings_orthosis.rise_time,...
    settings_orthosis.peak_time,...
    settings_orthosis.peak_time + settings_orthosis.fall_time]*1e2;

idx = (mesh_control >= t_pts(1)) & (mesh_control <= t_pts(3));

S = @(x) 10*x.^3 - 15*x.^4 + 6*x.^5;                                        % function with zero derivative (& 2nd derivative) at end points, and goes through 3 points perfectly

T = @(t) peak_torque .* ( ...
    (t>=t_pts(1) & t<=t_pts(2)).*S((t-t_pts(1))/(t_pts(2)-t_pts(1))) + ...
    (t>t_pts(2)  & t<=t_pts(3)).*S((t_pts(3)-t)/(t_pts(3)-t_pts(2))) );

Tspline(3, idx) = T(mesh_control(idx));

%% Apply exo torque on femur and tibia
exo.addBodyMoment(Tspline, ['T_ankle_exo_shank_',side],[upper_body_name,'_',side]);
exo.addBodyMoment(-Tspline, ['T_ankle_exo_foot_',side],[lower_body_name,'_',side],[upper_body_name,'_',side]);

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
        plot(mesh_control/N_stride*100,-Tspline(3,:),'DisplayName',legName)
        xlabel('Stride [%]')
        ylabel(["Assistance [Nm]";"<-- Extension Flexion -->"])
        title('ankleExoBryan2020')
        legend('Location','best')

    end

end

end

function w = smooth_window(t, t_on, t_off, eps)

    S_on  = 0.5*(1 + tanh((t - t_on)/eps));
    S_off = 0.5*(1 - tanh((t - t_off)/eps));
    
    w = S_on .* S_off;

end