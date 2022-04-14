# Usage

## Running a Simple Example

To run a simple example script, first activate your Pipenv environment.
```bash
cd ~/iitchs_base
pipenv shell
```
This will bring up a new shell with an activated virtualenv environment. You should see the letters `(iitchs_base)` at the front of your bash prompt.

Next, navigate to `iitchs_base/src` and run the `main.py` script with the `case1` example:
```bash
cd ~/iitchs_base/src
python main.py run catl_planning.examples.case1
```
You should begin seeing output printed to the screen, and then a Matplotlib image should appear.

To exit the program, click anywhere on the figure or press any key.

## Command Line Options

Case files can be called under two scenarios:
* `run` (e.g. `python main.py run catl_planning.examples.case1`
* `decomp` (e.g. `python main.py decomp catl_planning.examples.decomp_test_small`)
  * Attempts to decompose the problem into sub-specifications with sub-teams. This may fail if any of the decompositions are infeasible.

In addition, different optimization solvers can be specified from the command line using the `--solver` flag. The `--solver` flag and solver option must be passed immediately after the `run` or `decomp` command and before `catl_planning.examples.case1`. The solver options currently supported are 
* `--solver scip` ([SCIP](https://www.scipopt.org/) solver)
* `--solver gurobi` ([Gurobi](https://www.gurobi.com/) solver using [Gurobipy](https://www.gurobi.com/documentation/9.1/quickstart_mac/cs_grbpy_the_gurobi_python.html))
* `--solver gurobi_cmd` ([Gurobi](https://www.gurobi.com/) solver using command line)
* `--solver pulp_cbc_cmd` ([CBC](https://github.com/coin-or/Cbc) solver using PuLP's built-in CBC binary)
* `--solver coin_cmd` ([CBC](https://github.com/coin-or/Cbc) solver; same effect as calling `--solver pulp_cbc_cmd`)

For example, calling the `run` scenario using Gurobi can be done with the following command:
```bash
python main.py run --solver gurobi catl_planning.examples.case1
```
Calling the `decomp` scenario using the SCIP solver:
```bash
python main.py decomp --solver scip catl_planning.examples.case1
```
All `--solver` options are case-insensitive; e.g. `gurobi` and `GUROBI` will both call the Gurobi solver.

If no `--solver` option is passed, the SCIP solver is used by default.


## Example Case Files

Six example case files are currently included:

* `case1.py`: Can be called using, e.g., `python main.py run catl_planning.examples.case1`.
* `cs_two_region.py`: Can be called using, e.g., `python main.py run catl_planning.examples.cs_two_region`. Consists of a very simple transition system with two states and two agents. 
* `cs_sailp_4_test1.py`: Can be called using, e.g., `python main.py run catl_planning.examples.cs_sailp_4_test1.py`. Consists of a transition system with ten states and four agent. 
* `decomp_test.py`: Medium-sized example for decomposition, called with `python main.py decomp catl_planning.examples.decomp_test`
* `decomp_test_small.py`: A very small example for decomposition on the two region world, called with `python main.py decomp catl_planning.examples.decomp_test_small`
* `AAAI_base_case.py`: A small example for decomposition, called with `python main.py decomp catl_planning.examples.AAAI_base_case`

