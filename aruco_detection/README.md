# Rover Aruco Marker Detection Setup Guide in [INSTALL.md](./INSTALL.md).

# Aruco Marker IDs for our URC Rover
We have 11 ArUco tags/IDs
- `1`: Start post for Auto Nav mission
- `2`: Post 1 for Auto Nav mission
- `3`: Post 2 for Auto Nav mission
- `4`: Top left corner of keyboard for Equipment Servicing mission
- `5`: Top right corner of keyboard for Equipment Servicing mission
- `6`: Bottom right corner of keyboard for Equipment Servicing mission
- `7`: Bottom left corner of keyboard for Equipment Servicing mission
- `8`: Top left corner of USB-C port for Equipment Servicing mission
- `9`: Top right corner of USB-C port for Equipment Servicing mission
- `10`: Bottom right corner of USB-C port for Equipment Servicing mission
- `11`: Bottom left corner of USB-C port for Equipment Servicing mission

------
# aruco_detection

ROS2 Wrapper for OpenCV Aruco Marker Tracking

This package depends on a recent version of OpenCV python bindings and transforms3d library:

```
pip3 install opencv-contrib-python transforms3d
```

## ROS2 API for the aruco_detection Node

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
* `/camera/image_raw` (`sensor_msgs.msg.Image`)
* `/camera/camera_info` (`sensor_msgs.msg.CameraInfo`)

Published Topics:
* `/aruco_poses` (`geometry_msgs.msg.PoseArray`) - Poses of all detected markers (suitable for rviz visualization)
* `/aruco_markers` (`aruco_detection_interfaces.msg.ArucoMarkers`) - Provides an array of all poses along with the corresponding marker ids

Parameters:
* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `image_topic` - image topic to subscribe to (default `/camera/image_raw`)
* `camera_info_topic` - Camera info topic to subscribe to (default `/camera/camera_info`)
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)

## Running Marker Detection

1. Using the launch file
```
ros2 launch aruco_detection aruco_recognition.launch.py
```
2. As a single ROS 2 node
```
ros2 run aruco_detection aruco_node
```

## Generating Marker Images

```
ros2 run aruco_detection aruco_generate_marker
```

Pass the `-h` flag for usage information: 

```
usage: aruco_generate_marker [-h] [--id ID] [--size SIZE] [--dictionary]

Generate a .png image of a specified maker.

optional arguments:
  -h, --help     show this help message and exit
  --id ID        Marker id to generate (default: 1)
  --size SIZE    Side length in pixels (default: 200)
  --dictionary   Dictionary to use. Valid options include: DICT_4X4_100,
                 DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50, DICT_5X5_100,
                 DICT_5X5_1000, DICT_5X5_250, DICT_5X5_50, DICT_6X6_100,
                 DICT_6X6_1000, DICT_6X6_250, DICT_6X6_50, DICT_7X7_100,
                 DICT_7X7_1000, DICT_7X7_250, DICT_7X7_50, DICT_ARUCO_ORIGINAL
                 (default: DICT_5X5_250)
```
