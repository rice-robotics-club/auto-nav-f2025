This is shared repo for Robotics AutoNav Team. 


### Resources
## GNSS
Simulating odometry in Gazebo:
https://roboticsknowledgebase.com/wiki/common-platforms/ros/ros-mapping-localization/#:~:text=Then%20you%20can%20simply%20launch,publishes%20nav_msgs%2FOdometry%20type%20messages.

## GNSS stimulation with Gazebo 
To use, in WSL terminal do the following after git cloning:
cd ~/auto-nav-f2025/Testing_gazebo_ros2_bridge
colcon build
source install/setup.bash
ros2 launch ros_gz_example_bringup diff_drive.launch.py

Gazebo and RViz should show up with an example rover. 

## Cameras
Wide Lens Camera
ELP VGA Wide Angle USB Camera Module with 170degree Fisheye Lens for Computer 480P Mini UVC USB2.0 Embedded Webcam Board CMOS OV7725 Lightburn PC Camera for Laptop, Raspberry Pi, Jetson Nano
https://www.amazon.com/ELP-170degree-Fisheye-640x480-Resolution/dp/B00VTHD17W/ref=sr_1_13?crid=2J8QT6TQVTO59&dchild=1&keywords=elp+170+cam%5D&qid=1635916570&s=electronics&sprefix=elp+170+c%2Celectronics%2C298&sr=1-13
