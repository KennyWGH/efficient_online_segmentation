cmake_minimum_required(VERSION 3.10)
project(advanced_segmentation)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    cv_bridge
    image_transport
    pcl_ros
    pcl_conversions
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
    INCLUDE_DIRS ros_interface
    DEPENDS PCL
)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
)


##############################################


add_executable(advanced_segmentation_node 
    ros_interface/advanced_segmentation_node.cpp
    core/advanced_segmentation.cpp
    core/sgmtt_utility.cpp
    core/sector.cpp
)

target_link_libraries(advanced_segmentation_node 
    PUBLIC
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
)