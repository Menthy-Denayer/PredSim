03/10/2026
- "PreProcessing" < "getSpecificTensions.m" returned 0.25 default to 0.70 default like PredSim v1.1.0
- "add_pred_sim_to_batch.m" changed 0 to 3 outputs to accomodate personalization
- "runPredSim.m" changes for extra outputs to accomodate personalization, and removed OpenSim logger line for parallel workers to find OpenSim
- "run_pred_sim.m" changes for extra outputs to accomodate personalization and added OpenSim line for parallel workers to find OpenSim
- "initializeSettings.m" added line to be able to load settings from file, instead of based only on subject name
- "PostProcessing" < "PostProcessing.m" added R as output for parallel workers to access
- "DefaultSettings" < "getDefaultSettings.m" added option to specify S.misc.subject_path yourself