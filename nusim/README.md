# nusim Description
A simulator and visualizer for a red NU turtlebot. This node provides a simulated robot environment and uses rviz2 for visualization.
* `ros2 launch nusim nusim.launch.xml` to see the robot and specified obstacles in rviz.
![](images/nusim1.png)

# nusim Node Details
nusim node parameters:
   - rate: frequency of timer callback (defaults to 200 Hz)
   - x0: initial x-coordinate of the robot (m)
   - y0: initial y-coordinate of the robot (m)
   - theta0: initial rotation of the robot (rad)
   - obstacles
       - x: list of x-coordinates of obstacles (m)
       - y: list of y-coordinates of obstacles (m)
       - r: radius of obstacles (m)
   - arena
       - x_length: length of the arena in the world x direction (m)
       - y_length: length of the arena in the world y direction (m)
   - input_noise (double): variance for the zero mean Gaussian noise
   - slip_fraction (double): wheel slippage
   - basic_sensor_variance (double): sensor variance
   - max_range (double): max range for detecting obstacles
   - draw_only (bool): only draw real obstacles and wall if true, do simulation as well if false
   Lidar parameters:
   - range_min (double): lidar range minimum
   - range_max (double): lidar range maximum
   - angle_increment (double): lidar angle increment
   - num_samples (int): number of samples collected by the lidar per timer iteration
   - resolution (double): resolution of lidar sensor measurements
   - noise_level (double): lidar sensor noise
   DiffDrive parameters:
   - wheel_radius (double): wheel radius
   - track_width (double): track width (distance between the wheels)
   - motor_cmd_max (int): maximum ticks for motor commands
   - motor_cmd_per_rad_sec (double): number of rad/s equivalent to 1 motor command tick
   - encoder_ticks_per_rad (double): number of encoder ticks per radian
   - collision_radius (double): radius for collision detection

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