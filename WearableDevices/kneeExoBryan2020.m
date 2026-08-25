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
%       - stiffness:        stiffness of the virtual spring in Nm/kg/deg
%       - peak_torque:      peak torque in Nm/kg
%       - peak_time:        timing of peak as % of stride
%       - rise_time:        rise time as % of stride
%       - fall_time:        fall time as % of stride
%       - damping_onset:    onset of virtual damper-based assistance
%       - damping_offset:   offset of virtual damper-based assistance 
%       - damping_delta:    active time of the virtual damper as % stride (deprecated)
%       - damping:          damping of the virtual damper in Nm/rad/s
%       - upper_body:       name of the upper body to apply torque on
%       - lower_body:       name of the lower body to apply torque on
%       - bodymass:         mass of the body to scale stiffness and peak torque
%       - delta_noise:      offset applied to the mesh control variable to mimic errors in gait detection
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
%
% Last edit by: Menthy Denayer
% Last edit date: 14/April/2026
%
% --------------------------------------------------------------------------

%% Import Casadi
import casadi.*

%% Create Orthosis object
exo = Orthosis('exo',init,true);

%% Read settings that were passed from main.m
if isfield(settings_orthosis,'isFullGaitCycle')
    isFullGaitCycle = settings_orthosis.isFullGaitCycle;
else
    isFullGaitCycle = false;
end

%% Read Parameters
upper_body_name = settings_orthosis.upper_body;
lower_body_name = settings_orthosis.lower_body;
side = settings_orthosis.left_right;                                            % 'l' for left or 'r' for right

%% Unscale Variables
stiffness = settings_orthosis.stiffness * 180/pi * settings_orthosis.bodymass;  % [Nm/kg/deg] * [deg/rad] * [kg] = [Nm/rad]
peak_torque = settings_orthosis.peak_torque * settings_orthosis.bodymass;       % [Nm/kg] * [kg] = [Nm]

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

%% Add noise to mesh control
if(isfield(settings_orthosis,"delta_noise"))
    mesh_control = mesh_control + settings_orthosis.delta_noise;
end

%% Initialize torque profile
T_knee = SX.zeros(3, N_control);
eps_heavyside = 1;

%% Fill spring assistance
% get joint angles
q_knee = exo.var_coord([settings_orthosis.joint_name,side]);                    % knee angle in rad

% define torque
w_spring = smooth_window(mesh_control, settings_orthosis.stiffness_onset*1e2, settings_orthosis.stiffness_offset*1e2, eps_heavyside);

T_spring = -stiffness*q_knee.*w_spring;
T_spring_pos = 0.5 + 0.5*tanh(10*T_spring);
T_spring = T_spring .* T_spring_pos;                                            % once the knee angle hits zero, the torque remains at zero
T_knee(3, :) = T_knee(3, :) + T_spring;

%% Fill damping assistance
% get joint angular velocities
qdot_knee = exo.var_coord([settings_orthosis.joint_name,side],'vel');           % knee angular velocity in rad/s

% define torque
% damping_offset = settings_orthosis.damping_onset+settings_orthosis.damping_delta;
% if(damping_offset > 0.999)
%     damping_offset = 0.999;                                                   % clamp damping offset to end of gait cycle
% end

w_damp = smooth_window(mesh_control, settings_orthosis.damping_onset*1e2, settings_orthosis.damping_offset*1e2, eps_heavyside);
T_damp = -settings_orthosis.damping*qdot_knee.*w_damp;
T_knee(3, :) = T_knee(3, :) + T_damp;

%% Fill polynomial assistance
% define spline control points
t_pts = [settings_orthosis.peak_time*1e2 - settings_orthosis.rise_time*1e2, ...
         settings_orthosis.peak_time*1e2, ...
         settings_orthosis.peak_time*1e2 + settings_orthosis.fall_time*1e2];

% Evaluate spline only in active region
Tspline = zeros(3,N_control);
idx_spline = (mesh_control >= t_pts(1)) & (mesh_control <= t_pts(3));

% functions to generate spline
S = @(x) 10*x.^3 - 15*x.^4 + 6*x.^5;                                            % function with zero derivative (& 2nd derivative) at end points, and goes through 3 points perfectly

T = @(t) peak_torque .* ( ...
    (t>=t_pts(1) & t<=t_pts(2)).*S((t-t_pts(1))/(t_pts(2)-t_pts(1))) + ...
    (t>t_pts(2)  & t<=t_pts(3)).*S((t_pts(3)-t)/(t_pts(3)-t_pts(2))));

% assign torque
Tspline(3, idx_spline) = -T(mesh_control(idx_spline));

%% Total knee torque
T_tot = T_knee + SX(Tspline);

%% Apply exo torque on femur and tibia
exo.addBodyMoment(-T_tot, ['T_exo_thigh_',side],[upper_body_name,'_',side]);
exo.addBodyMoment(T_tot, ['T_exo_shank_',side],[lower_body_name,'_',side],[upper_body_name,'_',side]);
% exo.addBodyMoment(-T_tot, ['T_exo_thigh_',side],['exo_upper_leg_',side]);
% exo.addBodyMoment(T_tot, ['T_exo_shank_',side],['exo_lower_leg_',side],['exo_upper_leg_',side]);
% exo.addBodyMoment(-T_tot, ['T_exo_thigh_',side],['femur_',side]);
% exo.addBodyMoment(T_tot, ['T_exo_shank_',side],['tibia_',side],['femur_',side]);


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