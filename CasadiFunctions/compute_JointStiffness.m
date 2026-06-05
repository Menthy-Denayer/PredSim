function K_J = compute_JointStiffness(K_M, K_T, FT, FM, lMT, lTtilde, lMtilde, dM, drij_dtheta, model_info, lMo_in,...
    lTs_in, alphao_in, MuscMoAsmp)

% import casadi
import casadi.*

% define muscle properties
N_muscles = model_info.muscle_info.NMuscle;
lMo = ones(N_muscles,1).*lMo_in';
lTs = ones(N_muscles,1).*lTs_in';
alphao = ones(N_muscles,1).*alphao_in';

%% Define Variables
Njoints = model_info.ExtFunIO.jointi.nq.all;
lT = lTtilde .* lTs;
lM = lMtilde .* lMo;

%% Compute Pennation Angle
if(MuscMoAsmp == 0) % b = cst
    cos_alpha = (lMT-lT)./lM;
else    % alpha = cst = alphao
    cos_alpha = cos(alphao);
end

%% Compute Effective Muscle Stiffness
% use estimate of pennation angle, FM, lM to compute this part
K_Eff = K_M .* cos_alpha.^2 + FM./lM .* (1-cos_alpha.^2);

%% Compute MTU Stiffness
% use previous estimate and KT to do this part
K_MTU = 1./(1./K_T + 1./K_Eff);

%% Compute Stiffness Muscle-Joint
% K_M_J = K_MTU * rij² + drij/dtheta * F_T
K_MTU_3D = repmat(K_MTU, [1 Njoints]);
FT_3D    = repmat(FT,    [1 Njoints]);

term_elastic = K_MTU_3D .* dM.^2;
term_geometric = drij_dtheta .* FT_3D;

K_M_J = term_elastic + term_geometric;

%% Compute Joint Stiffness
% K_J = sum_M K_M_J

% K_J = zeros(1, Njoints);
K_J = SX.zeros(1,Njoints); % Joint stiffness
for j = 1:Njoints
    K_J(:,j) = sum(K_M_J(:,j), 1);
end

end