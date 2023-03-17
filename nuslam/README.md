# nuslam Description
The slam node implements an Extended Kalman Filter for SLAM on a NU turtlebot. The launch file launches the node as well as a simulator and visualizer.  

![](images/nuslam1.png)

The landmarks node implements clustering and circle-fitting algorithms for landmark detection from laser scan data. The launch file launches this node in addition to the slam launch file, with the parameter detect_landmarks set to true. A demo is shown later in this README.  

# slam Node Details
nuslam node parameters:  
    - body_id (string): body frame ID  
    - odom_id (string): odom frame ID  
    - wheel_left (string): left wheel frame ID  
    - wheel_right (string): right wheel frame ID  
    - wheel_radius (double): wheel radius  
    - track_width (double): track width (distance between the wheels)  
    - detect_landmarks (bool): whether or not the slam node should perform landmark detection

# slam Launch File Details
* `ros2 launch nuslam slam.launch.xml`
    
    Launch file arguments:  
    - cmd_src: source of cmd_vel commands  
        - defaults to none  
    - robot: robot to run on  
        - defaults to nusim  
    - use_rviz: whether or not to run rviz  
        - defaults to true  
    - detect_landmarks: whether or not to perform landmark detection  
        - defaults to false  

# landmarks Launch File Details
* `ros2 launch nuslam landmark_detect.launch.xml`

    Launch file arguments:  
    - cmd_src: source of cmd_vel commands  
        - defaults to none  
    - robot: robot to run on  
        - defaults to nusim  
    - use_rviz: whether or not to run rviz  
        - defaults to true  


Landmarks demo:  

https://user-images.githubusercontent.com/39091881/226039549-84e7545d-5706-473f-9c18-462cd17d56de.webm

The purple cylinders are the detected landmarks. The green cylinders are the obstacles as detected by SLAM (as a result of the detected landmarks). The red cylinders are the actual obstacles.  

Final positions from the actual robot (red), odometry (blue), and the slam estimate (green) in this demo:  
* actual robot (red): (0.101, 0.53123, 0)  
* odometry (blue): (0.099183, 0.531383, 0)  
* slam estimate (green): (0.026861, 0.67805, 0)  

Worked With: Meg Sindelar, Katie Hughes, Rintaroh Shima, Ritika Ghosh, Allan Garcia, Shantao Cao, Nick Morales, Liz Metzger, Marno Nel, Dilan Wijesinghe 