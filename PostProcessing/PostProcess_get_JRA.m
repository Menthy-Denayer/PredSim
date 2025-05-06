function [R] = PostProcess_get_JRA(model_info,f_casadi,R)
% --------------------------------------------------------------------------
% PostProcess_get_JRA
%   This function computes the joint reaction forces in the global frame
%   using the OpenSim joint reaction analysis tool.
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
%   * struct with simulation results, presumably with additional fields
% 
% Original author: Menthy Denayer
% Original date: 24/Aprl/2025
%
% Last edit by: Menthy Denayer
% Last edit date: 06/May/2025
% --------------------------------------------------------------------------

%% Load OpenSim Libraries
import org.opensim.modeling.*

%% Import OpenSim Model
% OpenSim model path
model_path = model_info.osim_path;

% import model
model = Model(model_path);
state = model.initSystem();

%% Write Muscle Forces to File
PostProcess_write_muscle_force_file(model_info,f_casadi,R);

%% Create Analyze Tool
% Add model
analyzeTool = AnalyzeTool(model);

%% Create Joint Reaction Analysis
% Computes joint reaction forces for all joints
jraTool = JointReaction();
jraTool.setName('JointReaction');
jraTool.setStartTime(R.time.mesh_GC(1));
jraTool.setEndTime(R.time.mesh_GC(end-1));

% add muscle forces (tendon force) file
forceFileName = fullfile(R.S.misc.save_folder,strcat(R.S.misc.result_filename,'_muscle_forces.sto'));
jraTool.setForcesFileName(forceFileName); 

% Choose frame
frame = ArrayStr();
frame.set(0, 'ground')      % express joint reaciton forces in ground
jraTool.setInFrame(frame)

% Set output options
jraTool.setOn(true);
jraTool.setInDegrees(true);

% Set start/end time of analysis
% limit to 1 gait cycle, as other outputs in R
analyzeTool.setInitialTime(R.time.mesh_GC(1));
analyzeTool.setFinalTime(R.time.mesh_GC(end-1));

% Set coordinates of model
% external forces do not need to be defined as they are computed by the
% ground contact spheres.
coordinatesFileName = fullfile(R.S.misc.save_folder,strcat(R.S.misc.result_filename,'.mot'));
analyzeTool.setCoordinatesFileName(coordinatesFileName);
analyzeTool.loadStatesFromFile(state);

% Add it to the AnalyzeTool
analyzeTool.getAnalysisSet().cloneAndAppend(jraTool);
analyzeTool.addAnalysisSetToModel();

%% Run Analysis 
% analyzeTool.print("JRA_settingsFile.xml")
analyzeTool.setName(R.S.misc.result_filename);
analyzeTool.setResultsDir(R.S.misc.save_folder);
analyzeTool.run();

%% Add Results to R Struct
jraFile = fullfile(R.S.misc.save_folder, strcat(R.S.misc.result_filename,'_JointReaction_ReactionLoads.sto'));
resultsTable = TimeSeriesTable(jraFile);

% Add column headers for computed joints
jraColumnHeaders = string(resultsTable.getColumnLabels().toArray())';
resHeaders = split(jraColumnHeaders,"_on"); resHeaders = resHeaders(:,1:9:end,1);   % for every joint, there are 9 data points (3x force, 3x moment & 3x point)
Njoints = length(resHeaders);
resHeaders = repelem(resHeaders,3) + repmat(["_x" "_y" "_z"],1,Njoints);            % add x, y, z for headers
R.colheaders.joint_reaction_forces = resHeaders;

% Add data to R struct
forceIdx = contains(jraColumnHeaders,"_in_ground_f");             % contact forces
momentIdx = contains(jraColumnHeaders,"_in_ground_m");            % contact moments
pointIdx = contains(jraColumnHeaders,"_in_ground_p");             % contact points
dataMatrix = resultsTable.getMatrix().getAsMat();
R.joint_reaction_forces.F = dataMatrix(:,forceIdx);
R.joint_reaction_forces.M = dataMatrix(:,momentIdx);
R.joint_reaction_forces.p = dataMatrix(:,pointIdx);











