% --------------------------------------------------------------------------
% test_PredSim
%   Run tests to ensure kinematics tracking works correctly.
%
%   Reference kinematics data taken from Van Criekinge et al., 2023
%       Van Criekinge, T., Saeys, W., Truijen, S. et al. A full-body motion 
%       capture gait dataset of 138 able-bodied adults across the life span 
%       and 50 stroke survivors. Sci Data 10, 852 (2023). 
%       https://doi.org/10.1038/s41597-023-02767-y
%
% Original author: Menthy Denayer
% Original date: 08 June 2025
%
% Last edit by: Menthy Denayer
% Last edit date: 06 January 2026 - changed weight settings to new format
% --------------------------------------------------------------------------

clear
close all
clc

% Check BLAS/LAPACK version; add functions from LinearAlgebra subdirectory
% to path in case Intel is *not* used
blas_version = version('-blas');
lapack_version = version('-lapack');
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

% cd(pathRepoFolder);

%% Parametersp
model_name = 'SUBJ09_gait1422';

%% Initialize S
addpath(fullfile(pathRepoFolder,'DefaultSettings'))

[S] = initializeSettings(model_name);

%% Settings

% name of the subject
S.subject.name = model_name;

% tracking kinematics
S.subject.TrackKin = true;
S.subject.TrackingFileKinematics = fullfile(pathRepo,'Subjects',S.subject.name,'SUBJ09-normalWalking-gait2334_noHead_pelvisOffset02_wToes_wSubtalar_newMarkers.mot'); 
S.subject.IncludeTrackingJoints = 'all';
S.subject.ExcludeTrackingJoints = 'mtp';
S.weights.kinematicsTracking = {{'all'},1e7,{'hip_adduction','pelvis_list'},5e7,{'ankle'},1e8};   
% S.weights.kinematicsTracking = {{'all'},1e7}; 

% tracking GRF
S.subject.TrackGRF = true;
S.subject.TrackingFileGRF = fullfile(pathRepo,'Subjects',S.subject.name,'SUBJ09-GRF-normalWalking-gait2334_noHead_pelvisOffset02_wToes_wSubtalar_newMarkers.sto'); 
S.subject.TrackingGRFs = {'ground_force_r_vx', 'ground_force_r_vy', 'ground_force_r_vz', 'ground_force_l_vx', 'ground_force_l_vy', 'ground_force_l_vz'};
% S.subject.TrackingGRFs = {'ground_force_r_vx', 'ground_force_r_vy', 'ground_force_l_vx', 'ground_force_l_vy'};
S.subject.TrackGRFexcludeSwing = false;                                             % include/exclude swing phase from GRF tracking
S.weights.grfTracking = {{'all'},1e2};

S.misc.gaitmotion_type = 'FullGaitCycle';                                           % required for selected mot file

% bushing penalty
S.subject.PenalizeBushings = false;
S.weights.bushings = 1;


% path to folder where you want to store the results of the OCP
S.misc.save_folder = fullfile(pathRepoFolder,'PredSimResults',[S.subject.name '_tracking']); 

% either choose "quasi-random" or give the path to a .mot file you want to use as initial guess
S.solver.IG_selection = fullfile(pathRepo,'Subjects',S.subject.name,'SUBJ09-normalWalking-gait2334_noHead_pelvisOffset02_wToes_wSubtalar_newMarkers.mot'); 
S.solver.IG_selection_gaitCyclePercent = 100;
S.solver.max_iter = 4e3;

% give the path to the osim model of your subject
osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);

% solver settings
S.solver.run_as_batch_job = false;

%% Run predictive simulations
[savename] = runPredSim(S, osim_path);