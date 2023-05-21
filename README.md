# Follow_me_algorithm
This repository will contain a follow me algorithm for the turtlebot3 burger model. 
Regulation_ang and Regulation_vel packages are the code for measuring the velocities over time both linear and angular.
The lidar_reg is on of the main packages for the system. This packages is the package containing the control systems for linear and angular velocities based on waypoint. In the folder the files used for the regulation tests can also be found. 

The waypoints of the calf comes from the package called shin_detector, all this package do is locating calves and publishing the waypoints of the calves. The package contains the trained txt files, with the feature extraction files. 
The package also contains the test file for calf detection test. 
