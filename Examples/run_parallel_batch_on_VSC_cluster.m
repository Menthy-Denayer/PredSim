% --------------------------------------------------------------------------
% run_parallel_batch_on_VSC_cluster
%   Run multiple PredSim simulations on the VSC cluster. KU Leuven provides 
%   compute resources to researchers in the High Performance Computing service. 
%   The HPC clusters of KU Leuven are part of the Vlaams Supercomputer 
%   Centrum (VSC).
% 
%   https://icts.kuleuven.be/sc/onderzoeksgegevens/hpc
%   https://www.vscentrum.be/
%
% Original author: Lars D'Hondt
% Original date: 01/October/2025
%
% Last edit by: Menthy Denayer
% Last edit date: 06 January 2026
% --------------------------------------------------------------------------

clear
close all
clc

%% Add PredSim Path
[pathExDir,~,~] = fileparts(mfilename('fullpath'));
[pathRepo,~,~] = fileparts(pathExDir);
[pathRepoFolder,~,~] = fileparts(pathRepo);

addpath(fullfile(pathRepo,'DefaultSettings'))
addpath(fullfile(pathRepo,'VariousFunctions'))
addpath(pathRepo)

%% Required Paths 
% Check BLAS/LAPACK version; add functions from LinearAlgebra subdirectory
% to path in case Intel is *not* used
blas_version = version('-blas');
lapack_version = version('-lapack');
if ~startsWith(lapack_version, 'Intel')
    % addpath(fullfile(getenv('PWD'), 'LinearAlgebra'))
    addpath(fullfile(pathRepo,'LinearAlgebra'))
end

addpath(fullfile(pathRepo,'DefaultSettings'))

% if the OpenSim module is loaded, make its Java library available
if isenv('EBROOTOPENSIM')
    javaclasspath(fullfile(getenv('EBROOTOPENSIM'), 'sdk', 'Java', 'org-opensim-modeling.jar'));
end

% if the CasADi-MATLAB module is loaded, expose its matlab bindings
if isenv('EBROOTCASADI')
    addpath(fullfile(getenv('EBROOTCASADI'), 'matlab'))
end

%% Import Libraries
% import org.opensim.modeling.*

% name of the subject
S.subject.name = 'SUBJ11_Dhondt2024_3seg';

%% Initialize S
[S] = initializeSettings(S.subject.name);
% [S] = initializeSettings(fullfile(pathRepo,'Subjects',S.subject.name,['settings_' S.subject.name '.m']));

%% Settings
% path to folder where you want to store the results of the OCP
S.misc.save_folder = fullfile(pathRepoFolder,'PredSimResults',S.subject.name,'weighted-LRposFixed'); 

% either choose "quasi-random" or give the path to a .mot file you want to use as initial guess
% S.solver.IG_selection = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_GC.mot');
S.solver.IG_selection = fullfile(pathRepo,'Subjects',S.subject.name,'SUBJ11-Dhondt2024_3seg-normalWalking_pelvisOffset0.mot')
S.solver.IG_selection_gaitCyclePercent = 100;
% S.solver.IG_selection = 'quasi-random';

% S.misc.gaitmotion_type = 'FullGaitCycle';

% cost function
% tracking joint stiffness
S.subject.TrackJointStiffness = false;
S.subject.TrackingFileJointStiffness = fullfile(pathRepo,'Subjects',S.subject.name,'SUBJ06_gait1422_11448691_zeroMTP.mot');
S.subject.IncludeTrackingJoints = 'all';
S.subject.ExcludeTrackingJoints = 'exo';
S.weights.jointStiffnessTracking = {{'all'},5e2}; 
% S.subject.AllowedJointStiffnessDeviation = 0.05;

% tracking kinematics
S.subject.TrackKin = false;
S.subject.TrackingFileKinematics = fullfile(pathRepo,'Subjects',S.subject.name,'gait1018_11676412-1GC.mot');
S.subject.IncludeTrackingJoints = 'all';
S.subject.ExcludeTrackingJoints = 'exo';
S.weights.kinematicsTracking = {{'all'},1e7};   

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

% create new model
osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);

% run predictive simulation
% [savename] = runPredSim(S, osim_path);

% give the path to the osim model of your subject
ref_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);

% solver settings
S.solver.run_as_batch_job = true;

%% Run predictive simulations
% foot_factor = linspace(0.5,1,10);                                               % scale foot mass by given factor
% Ncases = length(foot_factor);
% params.modelPath = ref_path;
% avg_vel = 0.6:0.1:2.0;

%% Weighted walking biological mass
% subj_mass = 65.64;                                                              % SUBJ04
% subj_mass = 73.99;                                                              % SUBJ06
% subj_mass = 55.91;                                                              % SUBJ07
% subj_mass = 64.22;                                                              % SUBJ09
% subj_mass = 50.27;                                                              % SUBJ10
subj_mass = 64.92;                                                              % SUBJ11

%% Weighted walking velocities
% avg_vel = [0.90, 0.87, 0.85, 0.90, 0.86];  % SUBJ04
% avg_vel = [1.17, 1.11, 1.12, 1.11, 0.99];  % SUBJ06
% avg_vel = [1.27, 1.29, 1.24, 1.22, 1.13];  % SUBJ07
% avg_vel = [1.19, 1.17, 1.16, 1.12, 1.10];  % SUBJ09
% avg_vel = [1.16, 1.13, 1.12, 1.03, 1.08];  % SUBJ10
avg_vel = [1.10, 1.04, 1.05, 1.01, 0.97];  % SUBJ11

Ncases = length(avg_vel);

% Sensitivity Analysis
% Ncases = 100;

% change subject path for neater file storage
S.misc.subject_path = fullfile(pathRepo,'Subjects',S.subject.name,'weighted');

c = parcluster;  
delete(c.Jobs)

for i = 1:Ncases
    % worker naming
    if(i < 10)
        trailingZero = '0';
    else
        trailingZero = '';
    end
    
    % use weighted model
    osim_path = fullfile(pathRepo,'Subjects',S.subject.name,'weighted',[S.subject.name '_weighted' num2str(i) 'kg.osim'])
    % osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '_' num2str(i) '.osim']);
    % adaptFootMass(foot_factor(i),params,osim_path)
    
    % choose speed
    S.misc.forward_velocity = avg_vel(i);
    
    % choose mass
    S.subject.mass = subj_mass;
    
    % interface parameters
    % new_path = fullfile(pathRepo,'Subjects',S.subject.name,'sensitivity_analysis',[S.subject.name '_' num2str(i) '.osim']);
    % adaptInterfaceParameters(ref_path, new_path)
    
    % run predictive simulation
    S.misc.result_filename = ['worker_' trailingZero num2str(i)];
    [savename] = runPredSim(S, osim_path);
end

c = parcluster;                                                         % retrieve job cluster
jobs = c.Jobs;                                                          % get jobs
arrayfun(@wait, jobs);                                                  % wait for all jobs to finish
delete(jobs);