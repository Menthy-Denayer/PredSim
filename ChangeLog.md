01/09/2026 by Menthy Denayer: Joint Stiffness computation
- Added 'compute_JointStiffness.m' function in 'CasadiFunctions' to compute the joint stiffness
- Added 'Ftparam.mat' in 'CasadiFunctions' to compute the joint stiffness
- Added 'ForceEquilibrium_dFtildeState_all_tendon.m' function in 'CasadiFunctions' to compute the partial derivative of the muscle force curves
- Added 'createCasadi_JointStiffness.m' function in 'CasadiFunctions' to initialize the Casadi joint stiffness functions
- Changed 'createCasadiFunctions.m' function in 'CasadiFunctions' to initialize joint stiffness Casadi functions
- Changed 'createCasadi_MSKGeometry.m' function in 'CasadiFunctions' to include dMdr as an output for the 'f_lMT_vMT_dM' function

- Added 'PostProcess_joint_stiffness.m' function in 'PostProcessing' to post-process the joint stiffness results
- Changed 'PostProcessing.m' function in 'PostProcessing' to run the joint stiffness post-processing

- Changed 'getDefaultSettings.m' function in 'DefaultSettings' to check whether to compute the joint stiffness

- NEED TO CHANGE: 'get_musculoskeletal_geometry_approximation.m' in 'PreProcessing' to make sure coefficients are always available for joint stiffness
- NEED TO CHANGE: added option in settings.misc to compute the joint stiffness or not, add as default setting (false)

25/08/2026 by Menthy Denayer
- Added Franks2020 exo assistance profiles
- Added line in "run_pred_sim.m", but it was commented. Would allow batch runs, but because of the logger command in "runPredSim.m" normal simulations don't find OpenSim libraries anymore.
- Made changes in the "Examples" < "VSC" folder to allow VUB HPC simulations to run (added "run_simulation_VUB.slurm" script, and changed "EBROOTCASADIMINMATLAB" to "EBROOTCASADI" in the "run_on_VSC_cluster.m" script)