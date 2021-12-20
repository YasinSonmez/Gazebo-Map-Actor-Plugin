cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(costmap_2d REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${tf2_ros_INCLUDE_DIRS})
include_directories(${costmap_2d_INCLUDE_DIRS})


link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${Boost_INCLUDE_DIRS})
link_directories(${tf2_ros_INCLUDE_DIRS})
link_directories(${costmap_2d_INCLUDE_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(MapActorPlugin
    SHARED
    MapActorPlugin.cc
#    plan_utils.cpp
)

target_link_libraries(MapActorPlugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${Boost_LIBRARIES} ${tf2_ros_LIBRARIES} ${costmap_2d_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")