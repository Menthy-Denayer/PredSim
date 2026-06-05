function [R] = PostProcess_joint_stiffness(model_info,f_casadi,R)
% --------------------------------------------------------------------------
% PostProcessing_joint_stiffness
%   Function to compute the joint stiffness for all joints
% 
% INPUT:
%   - model_info -
%   * structure with all the model information based on the OpenSim model
% 
%   - f_casadi -
%   * Struct containing all casadi functions.
%
%   - R -
%   * struct with simulation results
%
% OUTPUT:
%   - R -
%   * struct with joint stiffness field
% 
% Original author: Menthy Denayer
% Original date: 05/June/2026
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

%% Define Variables
a = R.muscles.a';
lMtilde = R.muscles.lMtilde';
lTtilde = R.muscles.lT ./[model_info.muscle_info.parameters.lTs];
lTtilde = lTtilde';
lMT = R.muscles.lMT';
vM = R.muscles.vM';
FT = R.muscles.FT';
FM = R.muscles.Fce'+R.muscles.Fpass';
Q = R.kinematics.Qs_rad';
rij = permute(R.muscles.dM, [2 3 1]);

%% Compute Muscle Stiffness
[KT, KM, ~] = f_casadi.f_muscle_tendon_stiffness(a,lMtilde,vM,FT);

KT_full = full(KT);
KM_full = full(KM);

R.joint_stiffness.KT = KT_full;
R.joint_stiffness.KM = KM_full;

%% Compute Derivative of Moment Arm
drdtheta = zeros(size(a,1),size(Q,1),size(Q,2));

for i = 1:size(drdtheta,3)
    drdthetaj = f_casadi.f_dr_dtheta(Q(:,i));
    drdtheta(:,:,i) = full(drdthetaj);
end

R.joint_stiffness.drdtheta = drdtheta;

%% Compute Joint Stiffness
KJ = zeros(size(Q));
R.joint_stiffness.KJ = KJ;

for i = 1:size(KJ,2)
    KJj = f_casadi.f_joint_stiffness(KM_full(:,i),KT_full(:,i),FT(:,i),FM(:,i),lMT(:,i),...
        lTtilde(:,i),lMtilde(:,i),rij(:,:,i),drdtheta(:,:,i));

    R.joint_stiffness.KJ(:,i) = full(KJj);
end