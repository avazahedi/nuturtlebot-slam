# ME495 Sensing, Navigation and Machine Learning For Robotics
* Ava Zahedi
* Winter 2023
# Package List
This repository consists of several ROS packages
- nuturtle_description - This package contains urdf files and basic debugging, testing, and visualization code for the WI2023 ME 495 robots.
- nusim - This package provides a simulated robot environment and uses rviz2 for visualization for a red NU turtlebot.
- nuturtle_control - This package enables control of the turtlebot via messages on the cmd_vel topic.

This repository also contains a custom C++ library
-  turtlelib - A library for handling transformations in SE(2) and other turtlebot-related math.
    * rigid2d -  vectors, twists, and transformations in 2D
    * diff_drive - kinematics for a differential drive robot in 2D