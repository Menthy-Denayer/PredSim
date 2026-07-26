function [R] = PostProcess_get_bushing_forces(model_info,f_casadi,R)
% --------------------------------------------------------------------------
% PostProcess_get_bushing_forces
%   This function calculates the bushing forces by evaluating the external
%   function for the optimal kinematics and adds the results to the struct
%   with results.
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
%   * struct with simulation results
% 
% Original author: Menthy Denayer
% Original date: 02/April/2022
%
% Last edit by: 
% Last edit date: 
% --------------------------------------------------------------------------

N = size(R.kinematics.Qs,1);

import casadi.*

F  = external('F',replace(fullfile(R.S.misc.subject_path,R.S.misc.external_function),'\','/'));

Foutk_opt = zeros(N,F.nnz_out);

for i = 1:N
    % Create zero input vector for external function
    F_ext_input = zeros(model_info.ExtFunIO.input.nInputs,1);
    % Assign Qs
    F_ext_input(model_info.ExtFunIO.input.Qs.all,1) = R.kinematics.Qs_rad(i,:);
    % Assign Qdots
    F_ext_input(model_info.ExtFunIO.input.Qdots.all,1) = R.kinematics.Qdots_rad(i,:);
    % Assign Qdotdots (A)
    F_ext_input(model_info.ExtFunIO.input.Qdotdots.all,1) = R.kinematics.Qddots_rad(i,:);

    % Evaluate external function
    res = F(F_ext_input);

    Foutk_opt(i,:) = full(res);
end

Nbushings = length(fieldnames(model_info.ExtFunIO.BUSHINGs))/2;
for i = 1:Nbushings
    R.bushing_forces.body1.("bushing_body1_" + i) = Foutk_opt(:,model_info.ExtFunIO.BUSHINGs.("bushing_body1_" + i));
    R.bushing_forces.body2.("bushing_body2_" + i) = Foutk_opt(:,model_info.ExtFunIO.BUSHINGs.("bushing_body2_" + i));
end


