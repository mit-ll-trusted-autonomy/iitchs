# Installation

## Prerequisites

These instructions assume you are using Ubuntu 18.04 with the following tools already installed:
* Terminal with BASH shell
* Git
* Python 3.8
* Anaconda3 or Miniconda3
* [Antlr](https://www.antlr.org/) version 4.8. Instructions for installing Antlr can be found below.
* At least one of the following optimization solvers:
  - Gurobi. Follow the Gurobi [Quick Start Guide](https://www.gurobi.com/documentation/quickstart.html) to install Gurobi and set up a license.
  - SCIP. Instructions for obtaining a license and installing can be found on the [SCIP website](https://www.scipopt.org/).

These instructions also assume you have SSH keys set up with Github. (This is not strictly necessary, but it makes installation more convenient.)

## Installation

There are three steps to installing IITCHS:
1. Install ANTLR 4.8
2. Install iitchs_base using conda
3. Generate ANTLR Files


### Installing Antlr 4.8

First, get the Antlr 4.8 binary. The binary can be placed anywhere, but note the name of the directory where it is installed. This tutorial will assume the binary is located in `~/antlr`.
```bash
mkdir ~/antlr
cd ~/antlr
wget https://www.antlr.org/download/antlr-4.8-complete.jar
```


Next, put the following lines in your .bashrc file, replacing `~/antlr` with the path to where you downloaded the Antlr binary.
```bash
export CLASSPATH=".:~/antlr:$CLASSPATH"
alias antlr4='java -jar ~/antlr/antlr-4.8-complete.jar -visitor'
alias grun='java org.antlr.v4.gui.TestRig'
```

After saving the file, source your .bashrc.
```bash
source ~/.bashrc
```


### Installing IITCHS_base with Conda

Make sure your system is up-to-date.
```bash
sudo apt update
```

Ensure the following packages are installed via apt:
```
sudo apt install libgmp3-dev
```

Clone the `iitchs_base` repository and its submodules. Be sure to use the `--recurse-submodules` option.
```bash
git clone --recurse-submodules git@github.mit.edu:iitchs/iitchs_base.git

```

Next we want to create a conda environment with the correct packages installed. We do this using the `environment.yml` file. Run the following command to generate the correct anaconda environment.  
```
cd <path to iitchs_base>
conda env create -f environment.yml
```
Next, we need to set the python path variables within the environment. This is done using:
```
conda activate iitchs_base
bash setup_conda_env_vars.sh
```
This will make two files in `/anaconda3/envs/iitchs_base/etc/conda/` that set your `$PYTHONPATH` when you activate and deactivate the env. If you want to add to these files, they augment the `~/.bashrc` when the environment is activated. These augmentations disappear when the environment is deactivated.

Next, deactivate and activate your conda environment to update your `$PYTHONPATH` with the required paths:
```
conda deactivate
conda activate iitchs_base
```

### Generate ANTLR Files

Finally, several files will need to be generated using Antlr 4.8. Navigate into the `iitchs_base` folder and run the following commands:
```bash
cd <path to iitchs_base>/src/python_stl/stl
antlr4 -Dlanguage=Python3 stl.g4
cd <path to iitchs_base>/src/catl/catl
antlr4 -Dlanguage=Python3 catl.g4
```
**Note:** Make sure that the "P" in "Python3" is capitalized, or Antlr will throw an error.





