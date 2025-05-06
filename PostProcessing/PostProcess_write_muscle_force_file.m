function [R] = PostProcess_write_muscle_force_file(model_info,f_casadi,R)
% --------------------------------------------------------------------------
% PostProcess_write_muscle_force_file
%   This function creates a muscle forces file with 2 steps of 
%   predicted gait.
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
% Original author: Lars D'Hondt (PostProcess_write_motion_file.m)
% Original date: 12/May/2022
%
% Last edit by: Menthy Denayer
% Last edit date: 06/May/2025
% --------------------------------------------------------------------------

%% Define Time Vector
% Two gait cycles
t_mesh = [R.time.mesh_GC(1:end-1),R.time.mesh_GC(1:end-1)+R.time.mesh_GC(end)];

%% Define Muscle Activations
% Muscle activations 
FT_GC = R.muscles.FT;
FT_GC_GUI = [t_mesh',[FT_GC;FT_GC]];

% Combine labels time and muscle activations
MuscleForce.labels = [{'time'},model_info.muscle_info.muscle_names];

%% Define Torque Actuated Joints
TJ_GC = R.torque_actuators.T;
TJ_GC_GUI = [TJ_GC;TJ_GC];

% Define torque actuated joint labels
TorqueActuator.labels = strcat('actuator_',R.colheaders.coordinates(model_info.ExtFunIO.jointi.torqueActuated));

%% Create Data Struct
% JRA tool in OpenSim requires radians
ActuatorData.inDeg = 'no';

% Combine data joint angles and speeds
ActuatorData.data = [FT_GC_GUI TJ_GC_GUI];
ActuatorData.labels = [MuscleForce.labels,TorqueActuator.labels'];

%% Create File
filenameMusclForce = fullfile(R.S.misc.save_folder, strcat(R.S.misc.result_filename,'_muscle_forces.sto'));
write_motionFile_v40(ActuatorData, filenameMusclForce);

%% Debug
% disp("Written muscle forces to: " + filenameMusclForce)

