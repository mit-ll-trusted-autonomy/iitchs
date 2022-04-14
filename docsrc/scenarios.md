# Creating Custom Scenarios.

Custom scenarios can be run by creating custom scenario files (sometimes called "casefiles"). 

<!-- When creating a custom scenario file, you will need a temporal logic formula specifying the desired actions for the agent team to complete. -->

## Structure of Scenario Files

Each scenario file must have the following elements:

**File Names**
* `ts_filename`: String. Path to a yaml file describing the transition system.
* `output_path`: String. Output path to the folder specifying where to save all IITCHS output file to, including. Output files include optimization `.lp`, `.mps`, and `.sol` files.
* `save_filename`: String. Output path **including the base filename** specifying where to save IITCHS output files. The base filename is used as the prefix for all output files.
The path (excluding the base filename) must match the path in `output_path`.
* `load_old_files`: Boolean. If true, IITCHS will load prior constraints / solution files.
* `reload_file`: String. The name for the file to reload if `load_old_files` is True

**Final Output Options**
* `sim_dynamics`: Boolean. If True, IITCHS will compute both the target path and a dynamical trajectory tracking it for each agent. The target path vs. dynamic trajectory will be plotted for each agent. The dynamics of each agent are specified using the parameters below.
* `sim_wheel_base`: Double.
* `k`: Double. Look-forward gain.
* `Lfc`: Double. Look-ahead distance.
* `Kp`: Double. Speed propportional gain.
* `dt`: Double. Time step in seconds.
* `L`: Double. Wheel base of each agent in meters.
* `kml_output`: Boolean. If true, the solution is put into a KML file which can be used to visualize the output using Google Maps.
* `corner_coor`: Tuple. The latitude/longitude coordinates of the bottom left corner of the transition system in the KML output.
* `txt_out`: Boolean. If true, IITCHS exports all waypoint data into individually named txt files.
<!-- * `show_animation`: Boolean. Creates a video of the agents dynamically tracking the target path. NOTE: Cannot currently be set from the case1.py file; this must be set directly in the pure_pursuit.py file.-->
<!-- * `old_nearest_point_index`: (?) Integer. -->

**Specification and Agents**
* `agents`: Python list. Specifies each agent's initial position and capabilities. The _i_th entry of the array contains a tuple describing the _i_th agent. The format of the tuple is `('initial_state', {'Capability1', 'Capability2',...})`.
* `formula`: String. Contains the CaTL formula to be satisfied by IITCHS. Will be parsed by Antlr.
* `show_visuals`: Boolean specifying whether to show the transition world before running the IITCHS routine. If true, a plot with each state and the graph lines connecting each state is shown.
* `robust`: Boolean.
* `regularize`: Boolean. If true, IITCHS regularizes the optimization travel time in order to reduce spurious motion.
* `alpha`: Double. Regularization parameter in the interval (0,1). Used if `regularize = True`.
* `upper_bound`: Boolean.
* `cap_height_map`: Numpy array.
* `agent_radius`: Numpy array. Collision avoidance radius for each agent


**Lower Level Trajectory Planning Variables**
* `local_obstacles`: Python list. List of obstacles considered when IITCHS plans actual trajectories, but not considered in the optimization.
The list should be in the following form:
```python
obstaclesList  = [[
        [0,0,0],
        [0,0,0]
    ],
    [
        [0,0,0],
        [0,0,0]
    ],
    [
        [0,0,0],
        [0,0,0]
    ]
    ]
```
The variable `obstaclesList` considers multiple separate planes of motion. Agents in different planes of motion do not collide. This may represent, for example, ground vehicles moving in one plane and aerial vehicles moving in a higher plane.
Notice that `obstaclesList` has the form `[outer_entry [inner entry [x,y,r],[x,y,r]]]`. Each outer entry defines a separate plane of motion and consists of multiple inner entries. Each inner entry consists of an obstacle specified by a circle with coordinates and radius `(x,y,r)`. There is no limit on the number of planes of motion that can be defined.
* `max_attempts`: Integer. Number of restarts allowed for one trajectory before restarting from scratch.
* `max_rrt_time`: Integer. Time allowed for single tree to be built before restart.
* `plot_region_bounds`: Python list. Reference frame bound for RRT algorithm. Has the form `[x_min, x_max, y_min, y_max]`.
* `world_max`: Python list. Has the form `[x_max, y_max]` with maximum `x`, `y` coordinates for the simulation world.
* `planning_step_time`: Integer. Number of steps in between each optimization step time.
* `show_sol`: Boolean. If True, a plot of all the final trajectories is shown after IITCHS finds a solution.
* `record_sol`: Boolean. If True, a video is created and saved showing all agents' trajectories being executed.
