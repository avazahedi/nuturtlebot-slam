# nusim  Description
A simulator and visualizer for a red NU turtlebot. This node provides a simulated robot environment and uses rviz2 for visualization.
* `ros2 launch nusim nusim.launch.xml` to see the robot and specified obstacles in rviz.
![](images/nusim1.png)

# Launch File Details
* `ros2 launch nusim nusim.launch.xml`

    Launch file arguments:
    - world_params: source of world parameters
        - defaults to nusim/config/basic_world.yaml
    - diff_params: source of DiffDrive parameters
        - defaults to nuturtle_description/config/diff_params.yaml
    - lidar_params: source of lidar parameters
        - defaults to nuturtle_description/config/lidar.yaml
    - rviz_config: rviz configuration file
        - defaults to nusim/config/nusim.rviz

Worked With: Meg Sindelar, Katie Hughes, Rintaroh Shima, Ritika Ghosh, Allan Garcia, Shantao Cao, Nick Morales, Liz Metzger, Marno Nel