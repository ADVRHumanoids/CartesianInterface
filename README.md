# Cartesian Interface
Package for generic cartesian control of floating base robots.
It includes a ROS-based front end, as well as a programmatic API
that can be used inside a real-time loop.

Have a look to the [**WIKI page**](https://github.com/ADVRHumanoids/CartesianInterface/wiki) 
for documentation!

## Quick start
You can choose between configuring the package via *ROS parameters* **or** via *YAML file*.
### Configuration via ROS
1) upload the robot URDF to `/robot_description` parameter
2) upload the robot SRDF to `/robot_description_semantic` parameter
3) upload the [cartesian control problem description](https://github.com/ADVRHumanoids/CartesianInterface/wiki/Get-started!#writing-an-ik-problem-for-your-robot) to the `/problem_description` parameter
4) if you just want to visualize the IK solution (no commands sent to the robot): 

    `rosparam set /xbotcore/cartesian/visual_mode true`
5) `roslaunch cartesian_interface cartesio.launch` runs both cartesian server and interactive marker spawner
### Configuration via YAML file
1) fill your *xbot config file* (`get_xbot_config` shell command) with a [cartesian control problem](https://github.com/ADVRHumanoids/CartesianInterface/wiki/Get-started!#writing-an-ik-problem-for-your-robot)
2) if you just want to visualize the IK solution (no commands sent to the robot): 
    `rosparam set /xbotcore/cartesian/visual_mode true`
    
3) `roslaunch cartesian_interface cartesio.launch use_xbot_config:=true` runs both cartesian server and interactive marker spawner

