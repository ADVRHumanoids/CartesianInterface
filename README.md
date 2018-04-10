# Cartesian Interface
Package for generic cartesian control of floating base robots.
It includes a ROS-based front end, as well as a programmatic API
that can be used inside a real-time loop.

Have a look to the [**WIKI page**](https://github.com/ADVRHumanoids/CartesianInterface/wiki) 
for documentation!

## Quick start
1) fill your *xbot config file* (`get_xbot_config` shell command) with a [cartesian control problem](https://github.com/ADVRHumanoids/CartesianInterface/wiki/Get-started!#writing-an-ik-problem-for-your-robot)
2) if you just want to visualize the IK solution (no commands sent to the robot): 

    `rosparam set /xbotcore/cartesian/visual_mode true`
3) run the ros server: `rosrun cartesian_interface ros_server_node`
