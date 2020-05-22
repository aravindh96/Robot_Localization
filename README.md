# Turtlebot Object Follower

This project demonstrates an object following custom turtlebot developed using ROS and Gazebo in a custom world environment.
The objective is achieved through 2 ROS Packages `ball_chaser` and `my_robot`. `ball_chaser` consists of 2 ROS nodes `\src\process_image.cpp`
and `\src\drive_bot.cpp` to create a service and call to command the robot based on camera input.

Developed ROS packages for custom robot visualization consisting of camera and Hokoyu Lidar using available plugins
Developed ROS nodes to process input image consisting of a subscriber and service client which calls the service `command_bot` 
to move the bot towards a white ball.


