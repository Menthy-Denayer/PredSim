function [f_muscle_tendon_stiffness,f_joint_stiffness] = createCasadi_JointStiffness(S,model_info)
% --------------------------------------------------------------------------
% createCasadi_JointStiffness
%   Function to create Casadi functions for joint stiffness.
%   
% INPUT:
%   - S -
%   * setting structure S
%
%   - model_info -
%   * structure with all the model information based on the OpenSim model
%
% OUTPUT:
%   - f_muscle_tendon_stiffness -
%   * function to compute the muscle & tendon stiffness
% 
% Original author: Ines Vandekerckhove, Tom Buurke & Dhruv Gupta, KU Leuven
% Original date: 30-11-2021 
%
% Last edit by: Menthy Denayer
% Last edit date: 01/September/2026 : Removed drdtheta function as included in lMT_vMT_dM Casadi function
% --------------------------------------------------------------------------

import casadi.*
N_muscles = model_info.muscle_info.NMuscle;
N_joints = model_info.ExtFunIO.jointi.nq.all;

%% Muscle & Tendon Stiffness
a          = SX.sym('a',N_muscles); % Muscle activations
lMtilde_in = SX.sym('lMtilde_in',N_muscles); % Muscle fibre lengths
vM         = SX.sym('vM',N_muscles); % Muscle fibre velocities
FT         = SX.sym('FT',N_muscles); % Tendon force
KM         = SX(N_muscles,1); % Muscle stiffness
KT         = SX(N_muscles,1); % Tendon stiffness
lTtilde    = SX(N_muscles,1); % Tendon stiffness

% Parameters of force-length-velocity curves
load('Ftparam.mat','Ftparam');
load('Fvparam.mat','Fvparam');
load('Fpparam.mat','Fpparam');
load('Faparam.mat','Faparam');

% Function to get muscle & tendons stiffness
for m = 1:N_muscles
    [KT(m), KM(m), lTtilde(m)] = ForceEquilibrium_dFtildeState_all_tendon(a(m),lMtilde_in(m),vM(m),FT(m),...
        model_info.muscle_info.parameters(m).FMo,model_info.muscle_info.parameters(m).lMo,...
        model_info.muscle_info.parameters(m).lTs,model_info.muscle_info.parameters(m).vMmax,...
        Ftparam,Fvparam,Fpparam,Faparam,...
        model_info.muscle_info.parameters(m).muscle_pass_stiff_shift,...
        model_info.muscle_info.parameters(m).muscle_pass_stiff_scale,model_info.muscle_info.parameters(m).muscle_strength,...
        model_info.muscle_info.parameters(m).tendon_stiff,...
        model_info.muscle_info.parameters(m).tendon_stiff_shift);
end
f_muscle_tendon_stiffness = ...
    Function('f_muscle_tendon_stiffness',{a,lMtilde_in,vM,FT},{KT, KM, lTtilde},...
    {'a','lMtilde','vM','FT'},...
    {'KT','KM','lTtilde'});


%% Joint Stiffness
KM_in       = SX.sym('KM_in',N_muscles); % Muscle stiffness
KT_in       = SX.sym('KT_in',N_muscles); % Tendon stiffness
drdtheta_in = SX.sym('drdtheta_in',N_muscles,N_joints); % Derivative of moment arm
FM          = SX.sym('FM',N_muscles); % Muscle force
lMT         = SX.sym('lMT',N_muscles); % Muscle-tendon length
lTtilde_in  = SX.sym('lTtilde_in',N_muscles); % Tendon length normalized
dM          = SX.sym('dM',N_muscles,N_joints); % moment arms

KJ = compute_JointStiffness(KM_in, KT_in, FT, FM, lMT, lTtilde_in, lMtilde_in, dM, drdtheta_in, model_info, ...
    [model_info.muscle_info.parameters.lMo],[model_info.muscle_info.parameters.lTs],...
    [model_info.muscle_info.parameters.alphao], S.misc.constant_pennation_angle);

f_joint_stiffness = ...
    Function('f_joint_stiffness',{KM_in,KT_in,FT,FM,lMT,lTtilde_in,lMtilde_in,dM,drdtheta_in},{KJ},...
    {'KM_in','KT_in','FT','FM','lMT','lTtilde_in','lMtilde_in','dM','drdtheta_in'},...
    {'KJ'});

end