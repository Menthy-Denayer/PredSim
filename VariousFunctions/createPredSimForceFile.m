clear all
clc
close all

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% AddPath
addpath("C:\Users\medenaye\Documents\programs\GitHub\OpenSim-Processing\data-processing\utilities");

%% Choose Results File
[resFile, resDIR] = uigetfile(".mat","Choose results file");
res = load(fullfile(resDIR,resFile));

%% Select Data
GRFr = res.R.ground_reaction.GRF_r;
GRFl = res.R.ground_reaction.GRF_l;
COPr = res.R.ground_reaction.COP_r;
COPl = res.R.ground_reaction.COP_l;

data = repmat([GRFr, COPr, GRFl, COPl],2,1);
data(101:end,[4,10]) = data(101:end,[4,10]) + res.R.kinematics.Qs(end,4);

time = [res.R.time.mesh_GC(1:end-1) res.R.time.mesh_GC(1:end-1)+res.R.time.mesh_GC(end)];

headers = ["ground_force_r_vx" "ground_force_r_vy" "ground_force_r_vz" "ground_force_r_px" "ground_force_r_py" "ground_force_r_pz" "ground_force_l_vx" "ground_force_l_vy" "ground_force_l_vz" "ground_force_l_px" "ground_force_l_py" "ground_force_l_pz"];

GRFtable = createTimeSeriesTable(headers,time,data);

mot_file = strrep(resFile,".mat","-GRF.mot");
STOFileAdapter().write(GRFtable, mot_file);

