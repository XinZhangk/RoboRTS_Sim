# Simple Simulation (for RoboRTS)
This is simply a skeleton project for multiple robot simulation. 
Currently the code is written *very* badly, with little documentation and much repetitions. 
Please improve the code quality and add more features.

## Run
* Put the package under the same `src` directory as `RoboRTS-v2` and `gazebo_temp` repositories.
* build the project
* open *four* terminals and run `source devel/setup.bash` for each of them
* at terminal1, run
```bash
roslaunch roborts_bringup multibots_sim.launch
```
* at terminal2, run
```bash
export ROS_NAMESPACE='/r1'
rosrun roborts_decision behavior_test_node
```
* at terminal3, run
```bash
 rosrun roborts_sim sim_node 
```
Note that terminal3 must *not* define ROS_NAMESPACE
* at terminal4, run
```bash
rosrun roborts_sim sim_cmd_node
```

Terminal1 takes care of launching of most of the nodes, including gazebo environment and navigation stacks.
Terminal2 brings up the decision node: currently only patrol mode is tested
Terminal3 starts the simulation node, which can take shooting command and keep track of hit points or ammunition. 
Terminal4 is supposed to be a weaponry system, equipped with commands issued by decision nodes. The current bare-bone version loops the shooting command to robot 2 on behalf of robot 1. As such, we disabled the ammo count and robot 1 effectively has infinite ammunition, for test purpose.

Also, when run rviz from r1's point of view, we can find a blue ray emanating from r1 toward r2 that indicates r1's line of sight (LOS).
When the r1's LOS is blocked by wall, the shooting will fail to deal damage.

## TODO
* add more functionality
* fix the issue that bullet will shoot through other robots
* integrate armor detection node, which works in simulation now ( thanks Guang)
* many other things I cannot think of or articulate as it is getting very late and I desperately need sleep