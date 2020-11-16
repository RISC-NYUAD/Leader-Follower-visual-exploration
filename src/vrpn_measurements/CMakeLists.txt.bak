cmake_minimum_required(VERSION 2.8.3)
project(vrpn_measurements)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf
    diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(CATKIN_DEPENDS 
    dynamic_reconfigure
    geometry_msgs
    message_runtime 
    roscpp 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(vrpn_log src/vrpn_log.cpp src/NE_utilities.cpp)
target_link_libraries(vrpn_log   ${catkin_LIBRARIES})

add_executable(vrpn_log_all src/vrpn_log_all.cpp src/NE_utilities.cpp)
target_link_libraries(vrpn_log_all   ${catkin_LIBRARIES})
