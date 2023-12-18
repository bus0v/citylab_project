This project is a ROS2 package that is meant to run on TurtleBot3. 
It is designed for the citylab simulation

![Robot-remote-real-robot-lab-simulation-2](https://github.com/bus0v/citylab_project/assets/51008991/f4d4271b-c265-4552-b182-6829bf3bc2e9)

The repo consists of
 * a custom service
 * a custom action
 * and various scripts

The action server takes in a desired global pose of the robot and drives to that destination.

The patrol service drives the robot around and avoids obstacles

The scan service determines which direction has the most open space.

The patrol with scan service drives the robot around while calling the scan service constantly.

