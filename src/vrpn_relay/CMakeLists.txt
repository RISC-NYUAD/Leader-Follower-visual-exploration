cmake_minimum_required(VERSION 2.8.3)
project(vrpn_relay)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wno-psabi ${CMAKE_CXX_FLAGS}")

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

add_executable(vrpn_relay src/vrpn_relay.cpp src/NE_utilities.cpp)
target_link_libraries(vrpn_relay   ${catkin_LIBRARIES})

add_executable(vrpn_relay_v2 src/vrpn_relay_v2.cpp src/NE_utilities.cpp)
target_link_libraries(vrpn_relay_v2   ${catkin_LIBRARIES})
