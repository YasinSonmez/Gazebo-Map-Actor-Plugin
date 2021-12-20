cmake_minimum_required(VERSION 3.0.2)
project(actor_pos_publish)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...

include_directories(./src/)

SET(actor_pos_publish_HEADERS src/actor_pos_publish_node.h src/plan_utils.h )
SET(actor_pos_publish_SOURCES src/actor_pos_publish_node.cpp src/plan_utils.cpp )

find_package(catkin REQUIRED COMPONENTS roscpp rospy gazebo_msgs actionlib std_msgs geometry_msgs genmsg sensor_msgs nav_msgs move_base_msgs tf2_ros tf2 tf2_geometry_msgs costmap_2d global_planner)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/actor_pos_publish_node.cpp src/plan_utils.cpp)

add_executable(actor_pos_publish_node ${actor_pos_publish_SOURCES} ${actor_pos_publish_HEADERS})

target_link_libraries(actor_pos_publish_node ${catkin_LIBRARIES} ${PROJECT_NAME})