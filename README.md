# GDP_gazebo
1.source devel/setup.bash
2.roslaunch m2wr_description gazebo_map.launch
3.source devel/setup.bash
4.roslaunch m2wr_description spawn.launch
5.source devel/setup.bash
6.rosrun camera_perception lane_detection_contour_gazebo.py
6.source devel/setup.bash
7.rosrun convert_velocities convertvelocitiesNode.py
9.source devel/setup.bash
10.python2 pid_controller.py
11.source devel/setup.bash
12.roslaunch m2wr_description rviz.launch

odometry
13.source devel/setup.bash
14.roslaunch my_odom_publisher start_odom.launch

IMU
To view the Imu data
15. Rostopic echo /rover/imu 
