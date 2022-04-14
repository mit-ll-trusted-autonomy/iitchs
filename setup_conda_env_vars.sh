
# Makes sure you're in the iitchs_base conda environment.
# It's a pain to undo all this if you run it
# accidentally in your base environment.
# 
# However, if you _really_ want to run this script in a
# different environment, call it with the -f argument.

# get directory of this script
SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

FORCE="false"

# Parse arguments
while getopts 'f' flag; do
	case "${flag}" in
		f) FORCE="true";;
	esac
done

if [ "$CONDA_DEFAULT_ENV" = "iitchs_base" ] || [ "${FORCE}" = "true" ]; then
	cd $CONDA_PREFIX
	
	mkdir -p ${CONDA_PREFIX}/etc/conda/activate.d
	
	mkdir -p ${CONDA_PREFIX}/etc/conda/deactivate.d
	
	touch ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	
	touch ${CONDA_PREFIX}/etc/conda/deactivate.d/env_vars.sh
	
	#These lines add the python path variables to the script
	echo "export PYTHONPATH_PRECONDA=\${PYTHONPATH}" > ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "export PYTHONPATH=\${PYTHONPATH}:${SCRIPTDIR}/src/python_stl" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "export PYTHONPATH=\${PYTHONPATH}:${SCRIPTDIR}/src/lomap" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "export PYTHONPATH=\${PYTHONPATH}:${SCRIPTDIR}/src/catl/catl" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "export PYTHONPATH=\${PYTHONPATH}:${SCRIPTDIR}/src/ltl2dstar-0.5.4/build" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "export PYTHONPATH=\${PYTHONPATH}:${SCRIPTDIR}/src" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "export PYTHONPATH=\${PYTHONPATH}:${SCRIPTDIR}/benchmarks" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	
	echo "export CLASSPATH=".:\${HOME}/antlr:\$CLASSPATH"" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "alias antlr4='java -jar \${HOME}/antlr/antlr-4.8-complete.jar -visitor'" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	echo "alias grun='java org.antlr.v4.gui.TestRig'" >> ${CONDA_PREFIX}/etc/conda/activate.d/env_vars.sh
	
	
	
	#This resets the pythonpath to its pre-conda condition after deactivation
	echo "export PYTHONPATH=\${PYTHONPATH_PRECONDA}" > ${CONDA_PREFIX}/etc/conda/deactivate.d/env_vars.sh
	echo "unalias antlr4" >> ${CONDA_PREFIX}/etc/conda/deactivate.d/env_vars.sh
	echo "unalias grun" >> ${CONDA_PREFIX}/etc/conda/deactivate.d/env_vars.sh
else
	echo "Please activate the 'iitchs_base' conda environment before running this script."
	echo "If you _really_ want to run it outside the 'iitchs_base' environment, use the -f flag."
fi
