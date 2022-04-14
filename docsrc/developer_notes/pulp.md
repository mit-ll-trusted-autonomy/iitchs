# The PuLP Modeler

PuLP is an LP modeler that can be used to create (mixed-integer) linear programs, and then solve them using a variety of different solvers as the backend.
IITCHS uses PuLP under the hood to create its MILP problems. This gives it the flexibility to switch between solvers specified by the user
at runtime.


## Official PuLP Resources
If you have questions about using PuLP, the following official resources are available:

* [Official documentation](https://coin-or.github.io/pulp/index.html)
* [Github Discussions page](https://github.com/coin-or/pulp/discussions)
* [Google Group](https://groups.google.com/g/pulp-or-discuss) (somewhat outdated, but useful for searching old answers. Use the Github Discussions page to ask new questions.)

The official PuLP documentation is unfortunately not very clear on certain details. The notes below contain additional information
on using PuLP that is helpful to know when working on the IITCHS codebase.


## Notes on Specific Solvers

### GUROBI_CMD

The `GUROBI_CMD` option calls the Gurobi command-line solver directly. Be aware that some attributes of the PuLP solver object do not get passed to
the `GUROBI_CMD` solver (e.g. the time limit). For best results, set the Gurobi parameters directly using the methods outlined below.

**Setting general Gurobi parameters:** General Gurobi parameters can be set using the `options` keyword argument when creating the solver object:

* The `options` variable must be a list of tuples, each tuple having length 2. 
* The first entry of a tuple in the `options` list must be the string name of the Gurobi parameter (e.g. "OutputFlag"). You can find a list of the parameter names
at the Gurobi [Parameters](https://www.gurobi.com/documentation/9.1/refman/parameters.html) webpage.
* The second entry of a tuple in the `options` list must be the value the parameter should have (e.g. 0 for "OutputFlag").

Example:
```python
import pulp
solver = pulp.GUROBI_CMD(options=[
	("OutputFlag", 0),
	("TimeLimit", 600)
])
```
A longer example can be found at [this link](https://groups.google.com/g/pulp-or-discuss/c/kzepw7kuFSA/m/f47LdLsxBAAJ).

**Setting the time limit:** Use the `options` keyword argument to set the "TimeLimit" parameter to the desired time limit (in seconds). **WARNING:** A PuLP solver object
(e.g. `solver = pulp.GUROBI_CMD()`) does have the attribute `timeLimit` (e.g. `solver.timeLimit`), but setting this attribute doesn't have any effect when using a 
`GUROBI_CMD` solver. Use the `options` method instead.

**Setting the verbosity:** Use the `options` keyword argument to set the "OutputFlag" parameter. A value of 1 implies full verbosity, and a value of 0 implies all output is turned off. You can see Gurobi's documentation for other verbosity parameters.

**Setting the number of threads:** The number of threads can be set using the `threads` keyword argument when creating the solver object:
```python
import pulp
solver = pulp.GUROBI_CMD(threads=1)
```

Alternatively, you can use the `options` keyword argument to pass in the "Threads" option value
```python
import pulp
solver = pulp.GUROBI_CMD(options=[
	("Threads", 1)
])

```
