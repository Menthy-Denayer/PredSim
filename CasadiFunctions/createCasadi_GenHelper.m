function [f_casadi] = createCasadi_GenHelper(S,model_info)
% --------------------------------------------------------------------------
% createCasadi_GenHelper
%   Function to create general Casadi functions.
%
% INPUT:
%   - S -
%   * setting structure S
% 
%   - model_info -
%   * structure with all the model information based on the OpenSim model
%
% OUTPUT:
%   - f_casadi -
%   * Struct that contains all casadi functions.
%
% Original authors: Dhruv Gupta, Lars D'Hondt, Tom Buurke
% Original date: 01/12/2021
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

import casadi.*

N_muscles = model_info.muscle_info.NMuscle;
N_arms_dof = model_info.ExtFunIO.jointi.nq.arms;
N_noarms_dof = model_info.ExtFunIO.jointi.nq.noArms;
N_torq_act = model_info.ExtFunIO.jointi.nq.torqAct;
N_pass_dof = model_info.ExtFunIO.jointi.nq.limTorq;

if S.subject.PenalizeBushings
    N_bushings = length(fieldnames(model_info.ExtFunIO.BUSHINGs))/2;            % added by Menthy
end

%% Normalized sum of squared values
if N_arms_dof > 0
    e_temp_arms_dof = SX.sym('e_temp_arms_dof',N_arms_dof);
    J_temp_arms_dof = 0;
    for i=1:length(e_temp_arms_dof)
        J_temp_arms_dof = J_temp_arms_dof + e_temp_arms_dof(i).^2;
    end
    J_temp_arms_dof = J_temp_arms_dof/N_arms_dof;
    f_casadi.J_arms_dof = Function('f_J_arms_dof',{e_temp_arms_dof},{J_temp_arms_dof});
end

if N_torq_act > 0
    e_temp_torq_act = SX.sym('e_temp_torq_act',N_torq_act);
    J_temp_torq_act = 0;
    for i=1:length(e_temp_torq_act)
        J_temp_torq_act = J_temp_torq_act + e_temp_torq_act(i).^2;
    end
    J_temp_torq_act = J_temp_torq_act/N_torq_act;
    f_casadi.J_torq_act = Function('f_J_torq_act',{e_temp_torq_act},{J_temp_torq_act});
end

e_temp_noarms_dof = SX.sym('e_temp_noarms_dof',N_noarms_dof);
J_temp_noarms_dof = 0;
for i=1:length(e_temp_noarms_dof)
    J_temp_noarms_dof = J_temp_noarms_dof + e_temp_noarms_dof(i).^2;
end
J_temp_noarms_dof = J_temp_noarms_dof/N_noarms_dof;
f_casadi.J_not_arms_dof = Function('f_J_not_arms_dof',{e_temp_noarms_dof},{J_temp_noarms_dof});

e_temp_pass_dof = SX.sym('e_temp_pass_dof',N_pass_dof);
J_temp_pass_dof = 0;
for i=1:length(e_temp_pass_dof)
    J_temp_pass_dof = J_temp_pass_dof + e_temp_pass_dof(i).^2;
end
J_temp_pass_dof = J_temp_pass_dof/N_pass_dof;
f_casadi.J_lim_torq = Function('f_J_lim_torq',{e_temp_pass_dof},{J_temp_pass_dof});

% Function for 2 elements
etemp2 = SX.sym('etemp2',2);
Jtemp2 = 0;
for i=1:length(etemp2)
    Jtemp2 = Jtemp2 + etemp2(i).^2;
end
Jtemp2 = Jtemp2/2;
f_casadi.J_2 = Function('f_J_2',{etemp2},{Jtemp2});

% Function for all muscles
e_temp_N_muscles = SX.sym('e_temp_N_muscles',N_muscles);
J_temp_N_muscles = 0;
for i=1:length(e_temp_N_muscles)
    J_temp_N_muscles = J_temp_N_muscles + e_temp_N_muscles(i).^2;
end
J_temp_N_muscles = J_temp_N_muscles/N_muscles;
f_casadi.J_muscles = Function('f_J_muscles',{e_temp_N_muscles},{J_temp_N_muscles});

%% Sum of squared values (non-normalized)
% Function for for distance between 2 points (in a certain plane)
e_temp_2 = SX.sym('e_temp_2',2);
J_temp_2 = 0;
for i=1:length(e_temp_2)
    J_temp_2 = J_temp_2 + e_temp_2(i).^2;
end
f_casadi.J_nn_2 = Function('f_J_nn_2',{e_temp_2},{J_temp_2});

%% Sum of squared values (non-normalized)
% Function for for distance between 2 points
e_temp_3 = SX.sym('e_temp_3',3);
J_temp_3 = 0;
for i=1:length(e_temp_3)
    J_temp_3 = J_temp_3 + e_temp_3(i).^2;
end
f_casadi.J_nn_3 = Function('f_J_nn_3',{e_temp_3},{J_temp_3});

%% Normalized sum of values to a certain power
% Function for number of muscles elements
e_temp_N_muscles_exp  = SX.sym('e_temp_N_muscles_exp',N_muscles);
expo        = SX.sym('exp',1);
J_temp_N_muscles_exp = 0;
for i=1:length(e_temp_N_muscles_exp)
    J_temp_N_muscles_exp = J_temp_N_muscles_exp + e_temp_N_muscles_exp(i).^expo;
end
J_temp_N_muscles_exp = J_temp_N_muscles_exp/N_muscles;
f_casadi.J_muscles_exp = Function('f_J_N_muscles_exp',{e_temp_N_muscles_exp,expo},{J_temp_N_muscles_exp});

%% Sum of products
% Function for number of muscles crossing a joint
sumCross = sum(model_info.muscle_info.muscle_spanning_joint_info);
N_musc_cross = setdiff(unique(sumCross),0);
for i = 1:length(N_musc_cross)
    ma_temp_musc_cross = SX.sym('ma_temp_musc_cross',N_musc_cross(i));
    ft_temp_musc_cross = SX.sym('ft_temp_musc_cross',N_musc_cross(i));
    J_sp_temp_musc_cross = 0;
    for j=1:length(ma_temp_musc_cross)
        J_sp_temp_musc_cross = J_sp_temp_musc_cross + ma_temp_musc_cross(j,1)*ft_temp_musc_cross(j,1);
    end
    f_casadi.(['musc_cross_' num2str(N_musc_cross(i))]) = Function(['musc_cross_' num2str(N_musc_cross(i))],{ma_temp_musc_cross,ft_temp_musc_cross},{J_sp_temp_musc_cross});
end

%% Kinematics Tracking
% Function for the number of joints to track
if S.subject.TrackKin || S.subject.TrackJointStiffness
    % check joints to track
    if(strcmp(S.subject.IncludeTrackingJoints,'all'))                      
        desir_coo_names = string(fieldnames(model_info.ExtFunIO.coordi));   % if tracking all joints
    else
        desir_coo_names = string(S.subject.IncludeTrackingJoints);          % if tracking only selected joints
    end
    
    % exclude joints to not track
    if(isfield(S.subject,'ExcludeTrackingJoints'))
        excludeBool = contains(desir_coo_names,S.subject.ExcludeTrackingJoints);
        desir_coo_names = desir_coo_names(~excludeBool);
    end
    
    % compute error
    NtrackJoints = length(desir_coo_names);                                 % number of joints to track
    e_temp_kin = SX.sym('NtrackJoints',NtrackJoints);                       % symbolic variable to store error (1xN joints to track)
    w_kin = SX.sym('w_kin',NtrackJoints);
    J_temp_kin = 0;                                                         % initialize cost 
    for i=1:length(e_temp_kin)                                              % loop over joints to track
        J_temp_kin = J_temp_kin + w_kin(i) * e_temp_kin(i).^2;
    end
    J_temp_kin = J_temp_kin/NtrackJoints;                                   % take average 
    f_casadi.J_kin = Function('f_J_kin',{e_temp_kin,w_kin},{J_temp_kin});   % create Casadi function
end

%% GRF Tracking
% Function for the number of GRFs to track
if S.subject.TrackGRF
    % compute error
    NGRF = length(S.subject.TrackingGRFs);                                  % number of joints to track
    e_temp_grf = SX.sym('Ngrfs',NGRF);                                      % symbolic variable to store error (1xN joints to track)
    w_mask = SX.sym('w_mask',NGRF);
    w_grf = SX.sym('w_grf',NGRF);
    J_temp_grf = 0;                                                         % initialize cost 
    for i=1:length(e_temp_grf)                                              % loop over joints to track
        J_temp_grf = J_temp_grf + w_grf(i) * w_mask(i) * e_temp_grf(i).^2;
    end
    J_temp_grf = J_temp_grf/NGRF;                                           % take average 
    f_casadi.J_grf = Function('f_J_grf',{e_temp_grf,w_mask,w_grf},{J_temp_grf});  % create Casadi function
end

%% Sum of squared bushing values (non-normalized)
% Function for bushing force penalty (added by Menthy)
if S.subject.PenalizeBushings
    p_bushing = SX.sym('p_bushing',N_bushings * 6);
    J_temp_bushings = 0;
    for i=1:length(p_bushing)
        J_temp_bushings = J_temp_bushings + p_bushing(i).^2;
    end
    f_casadi.J_bushings = Function('f_J_bushings',{p_bushing},{J_temp_bushings});
end

% %% Smooth window
% % Function to smooth variables instead of using if/else
% if isfield(S.subject,"AllowedJointStiffnessDeviation")
    
%     % check joints to track
%     if(strcmp(S.subject.IncludeTrackingJoints,'all'))                      
%         desir_coo_names = string(fieldnames(model_info.ExtFunIO.coordi));   % if tracking all joints
%     else
%         desir_coo_names = string(S.subject.IncludeTrackingJoints);          % if tracking only selected joints
%     end
    
%     % exclude joints to not track
%     if(isfield(S.subject,'ExcludeTrackingJoints'))
%         excludeBool = contains(desir_coo_names,S.subject.ExcludeTrackingJoints);
%         desir_coo_names = desir_coo_names(~excludeBool);
%     end
    
%     % compute weight
%     NtrackJoints = length(desir_coo_names);                                 % number of joints to track
%     a = SX.sym('a',NtrackJoints);                                           % symbolic variable to store input (1xN joints to track)
%     a_low = SX.sym('a_low',NtrackJoints);                                   % symbolic variable to store lower bound (1xN joints to track)
%     a_upp = SX.sym('a_upp',NtrackJoints);                                   % symbolic variable to store upper bound (1xN joints to track)
%     eps_hv = SX.sym('eps_hv',1);                                            % symbolic variable to store epsilon
    
%     S_on  = 0.5*(1 + tanh((a - a_low)/eps_hv));
%     S_off = 0.5*(1 - tanh((a - a_upp)/eps_hv));
    
%     w = 1 - S_on .* S_off;
    
%     f_casadi.f_smooth_window = Function('f_smooth_window',{a, a_low, a_upp, eps_hv},{w});
% end

end