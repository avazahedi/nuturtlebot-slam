# nuturtle_control  Description
Nodes for controlling the robot that are useful in both simulation and on the real robot.

# Launch File Details
* `ros2 launch nuturtle_control start_robot.launch.xml`
    Command line arguments:
    - cmd_src - source of cmd_vel messages
      - Options are [teleop, circle, none], defaults to "none" 
    - robot - choice of robot
      - Options are [nusim, localhost, none], defaults to "nusim"
    - use_rviz - whether or not rviz launches
      - Options are [true, false], defaults to "true"

Worked With: Meg Sindelar, Rintaroh Shima, Ritika Ghosh, Allan Garcia, Nick Morales, Katie Hughes, Liz Metzger, Dilan Wijesginhe, James Oubre, Muye Jia, Marno Nel