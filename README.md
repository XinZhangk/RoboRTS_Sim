# Simple Simulation (for RoboRTS)
This is simply a skeleton project for multiple robot simulation. 
Please improve the code quality and add more features.

Note that since Daniel kindly added the function to generate messages within the package, we need to run message generation first to create the corresponding header files.
At the workspace directory, run the following:
```bash
catkin_make roborts_sim_generate_messages
catkin_make sim_node
catkin_make sim_cmd_node
```
This will build all the necessary executables of simulation package.
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
ROS_NAMESPACE=/r1 rosrun roborts_decision behavior_test_node
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

## TODO (changed at 18th March)
* add definition for the callback functions
* fix the issue that bullet will shoot through other robots
* integrate armor detection node, which works in simulation now (thanks Guang)

After some discussion and reflection, it is clear that our previous direction is not suitable for a simulation 
upon which a decision node can be built for quick deployment on real-world robot.
The key issue is that we use different protocols, and if the simulation uses the customized protocols, then deploy our code 
to real robot will rise the issue of switching protocols.
For example, now we cannot ask simulation node that robot 1 wants to shoot robot 2.

The simulation also relies on a functioning gazebo model that is able to move gimbal, but on the correct model gimbal is
fixed on the chassis. 
An update on `gazebo_temp` will be added soon to support this feature.


# Simulation Road Map
Although we can bring up four different robots in the same gazebo environment and each of the robots has its own namespace and tf_prefix, an accurate simulation is still lacking.
To further illustrate the task, we shall inspect the current software structure of the real-world scenario.

![robot software structure](/docs/imgs/robot_control_physical.png) 

The key component that is replaced by the simulation is the `sensing/control` module.

To be more specific, the simulation should be able to listen to the command given by the local planner (such as `cmd_vel_acc`, which controls the robot's movement), chassis and gimbal executor (move chassis and gimbal as requested), detection (tracking enemy by moving camera alongside with gimbal).
On the other hand, simulation also has the responsibility to return information from odometry, camera and joints.

# Proposed Simulation Model
Our proposed simulation control contains three main components to replace the `sensing/control` module.
* `gazebo`: gazebo with its plugins are able to publish odometry information (by mimicking sensor readings), emulate camera input, and laser sensor input, and of course the modelling of the robot and map; the code base for this component is located at `gazebo_temp`, managed by guang hu.
* `sim_node`: simulation node plays the role of referee system and also takes commands from the sim base node.