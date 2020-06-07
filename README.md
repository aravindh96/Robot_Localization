# Turtlebot Object Follower

This project demonstrates an object following custom turtlebot developed using ROS and Gazebo in a custom world environment.
The objective is achieved through 2 ROS Packages `ball_chaser` and `my_robot`. `ball_chaser` consists of 2 ROS nodes `\src\process_image.cpp`
and `\src\drive_bot.cpp` to create a service and call to command the robot based on camera input.

Developed ROS packages for custom robot visualization consisting of camera and Hokoyu Lidar using available plugins
Developed ROS nodes to process input image consisting of a subscriber and service client which calls the service `command_bot` 
to move the bot towards a white ball.

## AMCL Parameter Tuning

1. __`max_particles`__ : Maximum Number of allowed particles
  > * Increasing this value increases computation but provides better estimates of localisation

2. __`laser_z_hit`__ : Mixture value for the weight of z_hit part of localization model. z_hit is represents the beams that hit an object
> * Increasing this value allows the laser scan to accurately estimate the obstacles to the actual obstacle position in the map. 
  
3. __`laser_z_rand`__ : Mixture value for weight of z_rand part of localization model. z_rand represents the uniform distribution used to model the situation when there may be some uncertainities.
> * Increasing this value reduces the ability of the bot to accurately update its estimation of the obstacle positions using the laser scan.


