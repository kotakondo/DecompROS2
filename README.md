### This is ported version for ROS2 humble of https://github.com/sikang/DecompROS 

#### Introduced lifetime for decomp_ros_msgs::msg::PolyhedronArray message type 

#### Dependencies

    pip install empy==3.3.4 catkin_pkg lark

#### To build 

    colcon build

#### To test 
    
    ros2 launch decomp_test_node test_decomp_rviz_launch.py

#### Build-related errors and solutions

##### Example 1

    Error: Could not find a package configuration file provided by "decomp_util" with any of the following names:

        decomp_utilConfig.cmake
        decomp_util-config.cmake

    Solution:

        source install/setup.bash

