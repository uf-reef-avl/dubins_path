cmake_minimum_required(VERSION 2.8.3)
project(dubins_path)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        rosflight_msgs
        reef_msgs
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)



catkin_install_python(PROGRAMS
        scripts/get_setpoint_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
