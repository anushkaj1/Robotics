# Robotics
Various robotics projects and tasks implemented, including SLAM and camera calibration.

Most of these were implemented in collaboration with [Himani Belsare](https://github.com/himanibelsare) and [Monica Surtani](https://github.com/Monica-Surtani) as part of the Mobile Robotics course offered at IIIT Hyderabad.

# Transformation and Mapping
- This task involves performing transformations on point clouds using angles and quaternions and visualising them using open3D.
- Further mapping is conducted by viewing a scene on AI2Thor and visualizing in open3D.

# ICP SLAM
- SLAM is implemented using the ICP algorithm.

# April Tags
- This project is made to detect April Tags in an environment. This can further allow for ease of implementing SLAM along with reduction in error.

## Project setup
This guide walks you through setting up and running a ROS project for detecting AprilTags using USB cameras.

### Prerequisites

- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- [usb_cam package](http://wiki.ros.org/usb_cam)
- apriltag library
- apriltag_ros library

### Getting Started

1. Clone the project repository:
   ```bash
   git clone https://github.com/Mobile-Robotics-IIITH-2023/project-outofmemor.git
   mkdir catkin_ws
    mv project-outofmemor.zip catkin_ws/
    cd catkin_ws
    unzip project-outofmemor.zip -d src
    rm project-outofmemor.zip
    catkin init

2. Install ROS packages:
    ```
    sudo apt-get install ros-melodic-perception
    sudo apt-get install ros-melodic-usb-cam
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install python3-catkin-tools
    export ROS_DISTRO=<ros_distro>
    source /opt/ros/$ROS_DISTRO/setup.bash
    ```
3. Install dependencies:
    ```
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
4. Build the workspace:
    ```
    catkin build
    ```

### Running the System

#### Terminal 1 - USB Camera Node
```bash
roslaunch usb_cam usb_cam-test.launch
```

#### Terminal 2 - AprilTag Detection Node
```
    roslaunch apriltag_ros continuous_detection.launch
```


#### Terminal 3 - Image Viewer
```
    rqt_image_view
```
#### Terminal 4 - RViz Configuration
```
    rviz
```

##### rviz config steps:
RViz Configuration Steps:
Click on the "Interact" icon on the left side of the screen.
Select "Image".
Navigate to "Displays" on the left.
Under "Image Topic", choose "/tag_detections.image".
Go to "Displays" -> "TF".
Under "Fixed Frame", select "usb_cam".
Now you can visualize the detected AprilTags.