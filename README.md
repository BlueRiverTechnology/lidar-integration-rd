# lidar-integration-rd
Packages to help tests methods for integrating lidar data

## Requirements
1. Ubuntu 20.04
1. ROS Noetic [Install Instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)
1. Catkin tools [Install Instructions](https://catkin-tools.readthedocs.io/en/latest/installing.html)
1. Rosdep [Install Instructions](http://wiki.ros.org/rosdep)

## Building ROS Workspace
1. Source ROS environment  
    ```
    source /opt/ros/noetic/setup.bash
    ```
1. Create Catkin workspace [Example here](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace)  
	```
	mkdir -p ~/ros/lidar_ws/src
	cd ~/ros/lidar_ws/
	catkin init
	```

1. Clone this repo in the **src** directory  
	```
    cd src
	git clone https://github.com/BlueRiverTechnology/lidar-integration-rd.git
	```
	
1. Get source dependencies
	```
	wstool init . lidar-integration-rd/.rosinstall
	```
1. Get debian dependencies
    ```
    rosdep install --from-path . --ignore-src
    ```
1. Build Catkin workspace
    ```
    catkin build
    ```
    At this point the code will begin to compile
1. Source the newly built workspace
    ```
    cd ..
    source devel/setup.bash
    ```
    
## Running field builder application
### Requirements
A ROS bag file(s) containing the following topics:
1. A sensor_msgs/PointCloud2 topic named **/ouster/points**
1. A sensor_msgs/NavSatFix topic named **/fix**
    
### Run the launch file
```
roslaunch field_building aggregate_clouds.launch bag_filename:=<full/path/to/bag/file.bag>
```
At this point rviz will run and the field will be incrementally built from the point clouds and gps data in the bag file
