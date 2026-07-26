% --------------------------------------------------------------------------
% Settings for Falisse_et_al_2022 that deviate from the PredSim defaults
%
%   A. Falisse, M. Afschrift, and F. D. Groote, “Modeling toes contributes 
%   to realistic stance knee mechanics in three-dimensional predictive 
%   simulations of walking,” PLOS ONE, vol. 17, no. 1, p. e0256311, 
%   Jan. 2022, doi: 10.1371/journal.pone.0256311.
%
% Original author: Lars D'Hondt
% Original date: 11/December/2023
% --------------------------------------------------------------------------

S.subject.name = 'Falisse_et_al_2022';

% S.misc.forward_velocity = 1.33;
S.misc.forward_velocity = 1.25;


S.subject.mtp_type = '2022paper';
S.subject.set_stiffness_coefficient_selected_dofs = {'mtp_angle',25};
S.subject.set_damping_coefficient_selected_dofs = {'mtp_angle',2};

% to prevent body segments from clipping into eachother
S.bounds.distanceConstraints(1).point1 = 'calcn_r';
S.bounds.distanceConstraints(1).point2 = 'calcn_l';
S.bounds.distanceConstraints(1).direction = 'xz';
S.bounds.distanceConstraints(1).lower_bound = 0.09;
S.bounds.distanceConstraints(1).upper_bound = 2;

S.bounds.distanceConstraints(2).point1 = 'hand_r';
S.bounds.distanceConstraints(2).point2 = 'femur_r';
S.bounds.distanceConstraints(2).direction = 'xz';
S.bounds.distanceConstraints(2).lower_bound = 0.18;
S.bounds.distanceConstraints(2).upper_bound = 2;

S.bounds.distanceConstraints(3).point1 = 'hand_l';
S.bounds.distanceConstraints(3).point2 = 'femur_l';
S.bounds.distanceConstraints(3).direction = 'xz';
S.bounds.distanceConstraints(3).lower_bound = 0.18;
S.bounds.distanceConstraints(3).upper_bound = 2;

S.bounds.distanceConstraints(4).point1 = 'tibia_r';
S.bounds.distanceConstraints(4).point2 = 'tibia_l';
S.bounds.distanceConstraints(4).direction = 'xz';
S.bounds.distanceConstraints(4).lower_bound = 0.11;
S.bounds.distanceConstraints(4).upper_bound = 2;

S.bounds.distanceConstraints(5).point1 = 'toes_r';
S.bounds.distanceConstraints(5).point2 = 'toes_l';
S.bounds.distanceConstraints(5).direction = 'xz';
S.bounds.distanceConstraints(5).lower_bound = 0.1;
S.bounds.distanceConstraints(5).upper_bound = 2;

% %% Add knee exoskeleton
% % select orthosis function
% exo1.function_name = 'kneeExoBryan2020';

% % average, optimal values Franks et al. (2020)
% exo1.stiffness_onset = 0.026;                                                    % [%]
% exo1.stiffness_offset = 0.267;                                                   % [%]
% exo1.stiffness = 0.009;                                                          % [Nm/kg/deg]
% exo1.peak_time = 0.59;                                                           % [%]
% exo1.rise_time = 0.214;                                                          % [%]
% exo1.fall_time = 0.095;                                                          % [%]
% exo1.peak_torque = 0.279;                                                        % [Nm/kg]
% exo1.damping_onset = 0.798;                                                      % [%]
% exo1.damping_offset = 0.969;                                                      % [%]
% exo1.damping = 2.176;                                                            % [Nm/rad/s]

% % general exo settings
% exo1.upper_body = 'femur';
% exo1.lower_body = 'tibia';
% exo1.joint_name = 'knee_angle_';
% exo1.isFullGaitCycle = true;
% exo1.bodymass = 62;

% % add orthosis on right side
% exo1.left_right = 'r';
% S.orthosis.settings{1} = exo1;

% % add the same orthosis on left side
% exo1.left_right = 'l';
% S.orthosis.settings{2} = exo1;
