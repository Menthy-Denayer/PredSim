% --------------------------------------------------------------------------
% run_iterative_on_VSC_cluster
%   Run multiple PredSim simulations on the VSC cluster, one after the other. 
% 
%   https://icts.kuleuven.be/sc/onderzoeksgegevens/hpc
%   https://www.vscentrum.be/
%
% Original author: Lars D'Hondt
% Original date: 01/October/2025
%
% Last edit by: Menthy Denayer
% Last edit date: 03 February 2026
% --------------------------------------------------------------------------

function run_iterative_batch_on_VSC_cluster(case_index)

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
    import org.opensim.modeling.*
    
    % name of the subject
    S.subject.name = 'SUBJ07opt3D';
    
    %% Initialize S
    [S] = initializeSettings(S.subject.name);
    % [S] = initializeSettings(fullfile(pathRepo,'Subjects',S.subject.name,['settings_' S.subject.name '.m']));
    
    %% Settings
    % path to folder where you want to store the results of the OCP
    S.misc.save_folder  = fullfile(pathRepoFolder,'PredSimResults',S.subject.name); 
    
    % worker naming
    if(case_index < 10)
        trailingZero = '0';
    else
        trailingZero = '';
    end
    S.misc.result_filename = ['worker_' trailingZero num2str(case_index)];
    
    % either choose "quasi-random" or give the path to a .mot file you want to use as initial guess
    % S.solver.IG_selection = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_GC.mot');
    if(case_index > 1)
        S.solver.IG_selection = fullfile(S.misc.save_folder, ['worker_' trailingZero num2str(case_index-1) '.mot']);
        S.solver.IG_selection_gaitCyclePercent = 200;
    else
        S.solver.IG_selection = fullfile(pathRepo,'Subjects',S.subject.name,'SUBJ07_gait1422_smallerToeContact_11399775-1GC.mot');
        S.solver.IG_selection_gaitCyclePercent = 100;
        % S.solver.IG_selection = 'quasi-random';
    end
    
    % create new model
    osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);
    
    % run predictive simulation
    % [savename] = runPredSim(S, osim_path);
    
    % give the path to the osim model of your subject
    ref_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);
    
    % solver settings
    S.solver.run_as_batch_job = false;
    
    %% Run predictive simulations
    % foot_factor = linspace(0.5,1,10);                                               % scale foot mass by given factor
    % Ncases = length(foot_factor);
    params.modelPath = ref_path;
    % avg_vel = 0.6:0.1:2.0;
    avg_vel = 1.2:-0.1:0.6;
    
    % Ncases = length(avg_vel);
        
    % create new model
    % osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '_' num2str(i) '.osim']);
    % adaptFootMass(foot_factor(i),params,osim_path)
        
    % choose speed
    S.misc.forward_velocity = avg_vel(case_index);
        
    % run predictive simulation
    [savename] = runPredSim(S, osim_path);
end