% --------------------------------------------------------------------------
% run_on_VSC_cluster
%   Run PredSim on the VSC cluster. KU Leuven provides compute resources to 
%   researchers in the High Performance Computing service. The HPC clusters 
%   of KU Leuven are part of the Vlaams Supercomputer Centrum  (VSC).
% 
%   https://icts.kuleuven.be/sc/onderzoeksgegevens/hpc
%   https://www.vscentrum.be/
%
% Original author: Lars D'Hondt
% Original date: 01/October/2025
% --------------------------------------------------------------------------

clear
close all
clc

% Check BLAS/LAPACK version; add functions from LinearAlgebra subdirectory
% to path in case Intel is *not* used
blas_version = version('-blas')
lapack_version = version('-lapack')
if ~startsWith(lapack_version, 'Intel')
    addpath(fullfile(getenv('PWD'), 'LinearAlgebra'))
end

[pathExDir,~,~] = fileparts(mfilename('fullpath'));
[pathRepo,~,~] = fileparts(pathExDir);
[pathRepoFolder,~,~] = fileparts(pathRepo);

addpath(fullfile(pathRepo,'DefaultSettings'))
addpath(pathRepo)

% if the OpenSim module is loaded, make its Java library available
if isenv('EBROOTOPENSIM')
    javaclasspath(fullfile(getenv('EBROOTOPENSIM'), 'sdk', 'Java', 'org-opensim-modeling.jar'));
end

% if the CasADi-MATLAB module is loaded, expose its matlab bindings
if isenv('EBROOTCASADI')
    addpath(fullfile(getenv('EBROOTCASADI'), 'matlab'))
end

% name of the subject
% S.subject.name = 'gait1018wFranks2020Exo';
S.subject.name = 'SUBJ06mtu3D';

%% Initialize S
[S] = initializeSettings(S.subject.name);

% path to folder where you want to store the results of the OCP
S.misc.save_folder  = fullfile(pathRepoFolder,'PredSimResults',S.subject.name); 

% either choose "quasi-random" or give the path to a .mot file you want to use as initial guess
% S.solver.IG_selection = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_GC.mot');
S.solver.IG_selection = fullfile(pathRepo,'Subjects',S.subject.name,'SUBJ06_gait1422_11448691_zeroMTP.mot')
S.solver.IG_selection_gaitCyclePercent = 100;
% S.solver.IG_selection = 'quasi-random';

% S.misc.gaitmotion_type = 'FullGaitCycle';

% give the path to the osim model of your subject
osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);

% tracking joint stiffness
S.subject.TrackJointStiffness = false;
S.subject.TrackingFileJointStiffness = fullfile(pathRepo,'Subjects',S.subject.name,'gait1018_12844352_noTorque_joint_stiffness.mot');
S.subject.IncludeTrackingJoints = 'all';
S.subject.ExcludeTrackingJoints = 'exo';
S.weights.jointStiffnessTracking = {{'all'},5e2}; 
% S.subject.AllowedJointStiffnessDeviation = 0.05;

% tracking kinematics
S.subject.TrackKin = false;
% S.subject.TrackingFileKinematics = fullfile(pathRepo,'Subjects',S.subject.name,'gait1018_11676412-1GC.mot');
S.subject.TrackingFileKinematics = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_GC.mot');
S.subject.IncludeTrackingJoints = 'all';
S.subject.ExcludeTrackingJoints = 'exo';
S.weights.kinematicsTracking = {{'all'},1e6};   

% tracking GRF
S.subject.TrackGRF = false;
S.subject.TrackingFileGRF = fullfile(pathRepo,'Subjects',S.subject.name,'gait1018_11676412-GRF.mot'); 
S.subject.TrackingGRFs = {'ground_force_r_vx', 'ground_force_r_vy', 'ground_force_r_vz', 'ground_force_l_vx', 'ground_force_l_vy', 'ground_force_l_vz'};
S.subject.TrackGRFexcludeSwing = false;                                             % include/exclude swing phase from GRF tracking
S.weights.grfTracking = {{'all'},1e2};

% bushing penalty
S.subject.PenalizeBushings = false;
S.weights.bushings = 0.25;

% negative muscle power penalty
S.subject.PenalizeNegMuscleWork = false;
S.weights.NegMuscleWork = 5;

% COM acceleration penalty
S.subject.PenalizeCOMacc = false;
S.weights.COMacc = 100;

% synergies
% S.subject.synergies = 1;
% S.subject.NSyn_r = 4;
% S.subject.NSyn_l = 4;

% slack controls
% S.weights.slack_ctrl = 1e5;

% set weights to zero to test out tracking
% S.weights.E = 0;
% S.weights.E_exp = 0;
% S.weights.q_dotdot = 0;
% S.weights.e_torqAct = 0;
% S.weights.pass_torq = 0;
% S.weights.pass_torq_includes_damping = 0;
% S.weights.a = 0;
% S.weights.a_exp = 0;
% S.weights.slack_ctrl = 0;

%% Run predictive simulations
[savename] = runPredSim(S, osim_path);

