% --------------------------------------------------------------------------
% Settings for gait1018 (i.e. 2D model) that deviate from the PredSim defaults
%
% Original author: Lars D'Hondt
% Original date: 12/August/2024
% --------------------------------------------------------------------------

S.subject.name = 'gait1018';
S.subject.mass = 62;
S.misc.forward_velocity = 1.25;                                                 % same as Franks2021

% This model has no arms
S.subject.base_joints_arms = []; 

% Achilles tendon stiffness
S.subject.tendon_stiff_scale = {{'soleus','gastroc'},0.5};

%% Add knee exoskeleton
% % select orthosis function
% exo1.function_name = 'kneeExoBryan2020';

% % set parameters of assistance profile
% exo1.stiffness_onset = 0.03;                                                    % [%]
% exo1.stiffness_offset = 0.25;                                                   % [%]
% exo1.stiffness = 0.0198;                                                        % [Nm/kg/deg]
% exo1.peak_time = 0.55;                                                          % [%]
% exo1.rise_time = 0.25;                                                          % [%]
% exo1.fall_time = 0.08;                                                          % [%]
% exo1.peak_torque = 0.196;                                                       % [Nm/kg]
% exo1.damping_onset = 0.75;                                                      % [%]
% exo1.damping_offset = 0.95;                                                     % [%]
% exo1.damping = 2;  

% % average, optimal values Franks et al. (2020)
% % exo1.stiffness_onset = 0.026;                                                    % [%]
% % exo1.stiffness_offset = 0.267;                                                   % [%]
% % exo1.stiffness = 0.009;                                                          % [Nm/kg/deg]
% % exo1.peak_time = 0.59;                                                           % [%]
% % exo1.rise_time = 0.214;                                                          % [%]
% % exo1.fall_time = 0.095;                                                          % [%]
% % exo1.peak_torque = 0.279;                                                        % [Nm/kg]
% % exo1.damping_onset = 0.798;                                                      % [%]
% % exo1.damping_offset = 0.969;                                                     % [%]
% % exo1.damping = 2.176;                                                            % [Nm/rad/s]

% % general exo settings
% exo1.upper_body = 'femur';
% exo1.lower_body = 'tibia';
% exo1.joint_name = 'knee_angle_';
% exo1.isFullGaitCycle = true;
% exo1.bodymass = S.subject.mass;

% % add orthosis on right side
% exo1.left_right = 'r';
% S.orthosis.settings{1} = exo1;

% % add the same orthosis on left side
% exo1.left_right = 'l';
% S.orthosis.settings{2} = exo1;

%% Add Hip Exoskeleton
% select orthosis function
exo1.function_name = 'hipExoBryan2020';

% Franks 2020 parameters
% exo1.rise_time_ext = 0.164;                                                     % [%]
% exo1.peak_time_ext = 0.262;                                                     % [%]
% exo1.peak_torque_ext = 0.404;                                                   % [Nm/kg]
% exo1.mid_time = 0.486;                                                          % [%]
% exo1.mid_duration = 0.016;                                                      % [%]
% exo1.peak_time_flex = 0.819;                                                    % [%]
% exo1.fall_time_flex = 0.247;                                                    % [%]
% exo1.peak_torque_flex = 0.286;                                                  % [Nm/kg]

% initial parameters
exo1.rise_time_ext = 0.150;                                                     % [%]
exo1.peak_time_ext = 0.260;                                                     % [%]
exo1.peak_torque_ext = 0.350;                                                   % [Nm/kg]
exo1.mid_time = 0.500;                                                          % [%]
exo1.mid_duration = 0.100;                                                      % [%]
exo1.peak_time_flex = 0.740;                                                    % [%]
exo1.peak_torque_flex = 0.350;                                                  % [Nm/kg]
exo1.fall_time_flex = 0.150;                                                    % [%]

exo1.upper_body = 'pelvis';
exo1.lower_body = 'femur';
exo1.isFullGaitCycle = true;
exo1.bodymass = S.subject.mass;

% add orthosis on right side
exo1.left_right = 'r';
S.orthosis.settings{1} = exo1;

% add the same orthosis on left side
exo1.left_right = 'l';
S.orthosis.settings{2} = exo1;

%% Add Ankle Exoskeleton
% % select orthosis function
% exo1.function_name = 'ankleExoBryan2020';

% % initial parameters
% exo1.peak_time = 0.48;                                                             % [%]
% exo1.rise_time = 0.30;                                                             % [%]
% exo1.fall_time = 0.10;                                                             % [%]
% exo1.peak_torque = 0.50;                                                           % [Nm/kg]

% % average, optimal values Franks et al. (2020)
% % exo1.peak_time = 0.550;                                                             % [%]
% % exo1.rise_time = 0.178;                                                             % [%]
% % exo1.fall_time = 0.195;                                                             % [%]
% % exo1.peak_torque = 0.935;                                                           % [Nm/kg]

% % general exo settings
% exo1.upper_body = 'tibia';
% exo1.lower_body = 'calcn';
% exo1.isFullGaitCycle = true;
% exo1.bodymass = S.subject.mass;

% % add orthosis on right side
% exo1.left_right = 'r';
% S.orthosis.settings{1} = exo1;

% % add the same orthosis on left side
% exo1.left_right = 'l';
% S.orthosis.settings{2} = exo1;