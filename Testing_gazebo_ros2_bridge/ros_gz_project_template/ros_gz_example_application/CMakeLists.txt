cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


install(PROGRAMS
  src/waypoint_publisher.py
  src/waypoint_subscriber.py
  src/waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()