function drij_dtheta = compute_drdtheta(q, model_info, nCoeffMat, expoVal_all)

%% Compute Derivative of Moment Arm
% compute derivatives at operating points
import casadi.*

Nmuscles = model_info.muscle_info.NMuscle;
Njoints = model_info.ExtFunIO.jointi.nq.all;
% drij_dtheta = zeros(Nmuscles, Njoints);
drij_dtheta   = SX.zeros(Nmuscles,Njoints);

for i = 1:Nmuscles

    isSpanning = model_info.muscle_info.muscle_spanning_joint_info(i,:);
    sampleIdxs = find(isSpanning);
    Ndof = length(sampleIdxs);
    qdof = q(sampleIdxs,1);

    order = model_info.muscle_info.polyFit.MuscleInfo.muscle(i).order;
    [~, ~, ddmat] = n_art_mat_3_cas_SX_7(qdof', nCoeffMat(order, Ndof), expoVal_all{order, Ndof});
    coeff = model_info.muscle_info.polyFit.MuscleInfo.muscle(i).coeff;

    for j = 1:Ndof
        drij_dtheta(i,sampleIdxs(j)) = -ddmat(:,j)' * coeff;

    end

end

end