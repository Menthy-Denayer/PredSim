#!/bin/bash -l

#SBATCH --cluster=hydra
#SBATCH --partition=zen4
#SBATCH --ntasks-per-node=2
#SBATCH --nodes=1
#SBATCH --time=01:00:00

# Instead of installing dependencies yourself, you can simply load modules on
# the cluster. The versions are just examples, you can check for available
# modules with a command like "module spider OpenSim"
module load OpenSim/4.5.2-foss-2024a-MATLAB-2024a-r6
module load CMake/3.29.3-GCCcore-13.3.0

# Prevent GitHub from asking for a password
export GIT_TERMINAL_PROMPT=0
export GIT_SSH_COMMAND="ssh -oBatchMode=yes"

# See https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=91&t=7165&p=36596
export LD_PRELOAD="${EBROOTGCCCORE}/lib64/libstdc++.so.6"

# Update the Java library path in order enable using the OpenSim Java bindings
export _JAVA_OPTIONS="${_JAVA_OPTIONS} -Djava.library.path=$EBROOTOPENSIM/sdk/lib"

# Let MATLAB pick up OpenBLAS from a module
export BLAS_VERSION=${EBROOTOPENBLAS}/lib64/libopenblas.so
export LAPACK_VERSION=${EBROOTOPENBLAS}/lib64/libopenblas.so

# TODO Figure out if it useful to use more than a single thread
export OMP_NUM_THREADS=1
matlab -nodisplay -nosplash -singleCompThread -r "addpath('Examples'); 	run_on_VSC_cluster"
