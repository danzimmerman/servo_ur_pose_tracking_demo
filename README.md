# servo_ur_pose_tracking_demo

Working attempt to run the MoveIt Servo pose tracking demo with UR robots.

See https://youtu.be/-5iBKqu-9CY

By default, the package uses fake/mock hardware and launches 
 - A ROS 2 Control controller manager
 - An arm controller for the fake hardware specified in the servo config YAML file
 - RViz for visualization.

 If you want to test the demo with real hardware or URSim, first, start `ur_robot_driver`, make sure `forward_position_controller` is loaded, and make sure the external control program is running on the robot to connect it to the driver.
 
 Then run:

 ```
 ros2 launch servo_ur_pose_tracking_demo pose_tracking_example.launch.py use_fake_hardware:=false 
 ``` 

To run this without RViz visualization, you can pass `launch_rviz:=false`.

