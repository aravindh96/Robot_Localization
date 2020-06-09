# Adaptive Monte Carlo Localization

This project demonstrates a robot localization using the __Adaptive Monte Carlo Localization__ algorithm. Localization task is implemented on a custom turtlebot having a Hokuyo laser scanner in a custom map built using __Gazebo__.

AMCL is a __particle filter localization__ algorithm
The Extended Kalman Filter(EKF) algorithm is restricted by a __linear Gaussian state based assumption__, this is not the case for AMCL as it can model any kind of distribution. AMCL is capable of solving the __local, global and kidnapped robot__ localization problems.

AMCL works by __uniformly and randomly distributing particles__ throughout the __known map__. These particles represent the potential positions of the robot and its orientation. Each particle has a __weight__ associated with it which determines the level of confidence or __probability of being the true position__. Through several iterations of the algorithm, as the robot moves around, the particles are updated with the respective motion, and the weights are updated by calculating __the error between the distance of each particle to a set of landmarks and the distance of the robot to the same set of landmarks.__ Based on these weights, the set of particles is resampled using `systematic resampling`. After a few iterations the number of particles will reduce and __eventually converge__ to provide a __confident estimate__ of the robot position.

<p align="center">
<img src="https://github.com/aravindh96/Robot_Localization/blob/master/Images/Top_view_map.png" alt="drawing" width="300"/> <img src="https://github.com/aravindh96/Robot_Localization/blob/master/Images/map.jpg" alt="drawing" width="300"/> 
</p>

<p align="center">
<img src="https://github.com/aravindh96/Robot_Localization/blob/master/Images/Top_view_map.png" alt="drawing" width="300"/> <img src="https://github.com/aravindh96/Robot_Localization/blob/master/Images/map.jpg" alt="drawing" width="300"/> 
</p>


## Prerequisites:
1. [Ubuntu](https://ubuntu.com/download)
2. [Robot Operating System](http://wiki.ros.org/ROS/Installation) (ROS)
3. Install ROS nodes
    ```bash
    $ sudo apt-get update
    $ sudo apt-get install ros-kinetic-move-base
    $ sudo apt-get install ros-kinetic-map-server
    $ sudo apt-get install ros-kinetic-amcl
    $ sudo apt-get install ros-kinetic-base-local-planner
    $ sudo apt-get install ros-kinetic-global-planner
    ```
---
## Implementation
1. Change directory to `catkin_ws/src`
2. Clone this repository
    ```
    $ git clone https://github.com/aravindh96/Robot_Localization.git
    ```
3. Build the project
    ```
    $ catkin ..
    $ catkin_make
    ```
4. Source the setup.bash file
    ```
    $ source devel/setup.bash
    ```
5. Run the launch files in two terminals
    ```
    $ roslaunch my_robot empty_world.launch
     "Open new terminal and source setup file"
    $ roslaunch my_robot amcl.launch
    ```
 
## RosNodes:
 1. [/map_server](http://wiki.ros.org/map_server): Locates the map that you created and send to other ROS nodes
 2. [/amcl](http://wiki.ros.org/amcl#Parameters): Node takes the sensor and map data to perform AMCL.
 3. [/move_base](http://wiki.ros.org/move_base): Implements navigation using local and global planners.
 
 

## AMCL Parameter Tuning

1. __`max_particles`__ : Maximum Number of allowed particles
  > * Increasing this value increases computation but provides better estimates of localisation

2. __`laser_z_hit`__ : Mixture value for the weight of z_hit part of localization model. z_hit is represents the beams that hit an object
> * Increasing this value allows the laser scan to accurately estimate the obstacles to the actual obstacle position in the map. 
  
3. __`laser_z_rand`__ : Mixture value for weight of z_rand part of localization model. z_rand represents the uniform distribution used to model the situation when there may be some uncertainities.
> * Increasing this value reduces the ability of the bot to accurately update its estimation of the obstacle positions using the laser scan.

4. __`laser_max_beams`__: Maximum number of evenly spaced beams to be used in each scan.
> * Increasing this improves the localization but also increases the computation, so choosing a balanced value of 50 provides the best results.

5. __`odom_alpha*`__: The four noise parameters for the rotational and translational movement of the robot.
> * By default these values are set to '0.20' but since we are using a simulated environment we can assume that there is no noise in the rotational and translational movement. As soon as this is changeed it provides the biggest improvement in localization which in turn improves the performance of the path planning and navigation steps.


## move_base Package

This package has been used to implement the navigation of the robot, which it achieves by linking a global planner and a local planner.

[Global Planner](http://wiki.ros.org/global_planner) : `global_planner/Global Planner` 
A fast interpolated global planner, a more flexible replacement for navfn.

[Local Planner](http://wiki.ros.org/base_local_planner) : `base_local_planner`
This package implements the Dynamic Window Approach (DWA) to achieve robot navigation. It creates a kinematic trajectory for the robot to move from start to finish.



