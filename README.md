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

### Generate the Octomap (height map)
1. Preferably wait until the application prints the following message `Max cloud count ??? reached, aggregation stopped;`

2. Call the **build_field_octomap** service by using the rosservice call command from the terminal
	```
	rosservice call /build_field_octomap "{}" 
	```
	
	> Hint: After entering most of the service name pressing the "Tab" key should autocomplete the rest	
3. Wait a little while and eventually the octomap display should show in Rviz.  Also the terminal from where the ROS service command was ran should print the response message.


## Open3D (Optional)
[Open3d](http://www.open3d.org/) is a library for rapid development of software that deals with 3D data.

### Installation 

1. Download the development library from https://github.com/isl-org/Open3D/releases.  This procedured was verified to work with the [open3d-devel-linux-x86_64-pre-cxx11-abi-cuda-0.16.0 version](https://github.com/isl-org/Open3D/releases/download/v0.16.0/open3d-devel-linux-x86_64-pre-cxx11-abi-cuda-0.16.0.tar.xz)

2. Extract the compressed file, inside the main directory there should be an "include" and "lib" directories

3. Create a soft link so that cmake can find the library
    ```
    ln -s </abs/path/to/library> /usr/local/open3d
    ```
    
### Build Open3D cloud aggregator in the catkin workspace
1. Pass the USE_OPEN3D flag to the catkin command as follows
    ```
    catkin build field_building --cmake-args -DUSE_OPEN3D=true
    ```
2. The Open3D version of the cloud aggregator application will be built.

### Running Open3D cloud aggregator
The Open3D cloud aggregator operates in the same way as the default aggregator, however it uses Open3D classes and functions instead of PCL(Point Cloud Library) as it is the case for the default aggregator.  In order to use it the `use_open3d` argument in the launch file needs to be set to `true`.  An example of how running the launch file in this way would be as follow:
    
    roslaunch field_building aggregate_clouds.launch use_open3d:=true bag_filename:=<full/path/to/bag/file.bag>
    

