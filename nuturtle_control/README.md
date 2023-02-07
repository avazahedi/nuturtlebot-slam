# nuturtle_control  Description
Nodes for controlling the robot that are useful in both simulation and on the real robot.
* `ros2 launch nuturtle_control start_robot.launch.xml`
<!-- ![](images/nusim1.png) -->

# Launch File Details
* `ros2 launch nuturtle_control start_robot.launch.xml`
    <!-- Parameters, which can be changed in config/basic_world.yaml:
    - rate: frequency of timer callback (defaults to 200 Hz)
    - x0: initial x-coordinate of the robot (m)
    - y0: initial y-coordinate of the robot (m)
    - theta0: initial rotation of the robot (rad)
    - obstacles
        - x: list of x-coordinates of obstacles (m)
        - y: list of y-coordinates of obstacles (m)
        - r: radius of obstacles (m) -->

Final location of the turtlebot according to odometry:
`header:
  stamp:
    sec: 1675804732
    nanosec: 622941451
  frame_id: odom
child_frame_id: blue/base_footprint
pose:
  pose:
    position:
      x: 11.189463871019605
      y: -31.1438468166467
      z: 0.0
    orientation:
      x: -0.0
      y: 0.0
      z: 0.8547107703828232
      w: -0.5191045164430772`


Worked With: Meg Sindelar, Rintaroh Shima, Ritika Ghosh, Allan Garcia, Nick Morales, Liz Metzger, Dilan Wijesginhe, James Oubre, Muye Jia, Marno Nel