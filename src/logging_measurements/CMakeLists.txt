cmake_minimum_required(VERSION 2.8.3)
project(logging_measurements)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf
    diagnostic_updater
    image_transport
    mavros_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(CATKIN_DEPENDS 
    dynamic_reconfigure
    geometry_msgs
    message_runtime 
    roscpp 
    image_transport
    mavros_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(vrpn_log src/vrpn_log.cpp src/NE_utilities.cpp)
target_link_libraries(vrpn_log   ${catkin_LIBRARIES})

add_executable(vrpn_log_all src/vrpn_log_all.cpp src/NE_utilities.cpp)
target_link_libraries(vrpn_log_all   ${catkin_LIBRARIES})

add_executable(rhombi_experiment src/rhombi_experiment.cpp src/NE_utilities.cpp)
target_link_libraries(rhombi_experiment   ${catkin_LIBRARIES})

add_executable(NYUNY_logger src/NYUNY_logger.cpp src/NE_utilities.cpp)
target_link_libraries(NYUNY_logger   ${catkin_LIBRARIES})

add_executable(Leader_Follower src/Leader_Follower.cpp src/NE_utilities.cpp)
target_link_libraries(Leader_Follower   ${catkin_LIBRARIES})

add_executable(NYUNY_logger_Daitao src/NYUNY_logger_Daitao.cpp src/NE_utilities.cpp)
target_link_libraries(NYUNY_logger_Daitao   ${catkin_LIBRARIES})

add_executable(NYUNY_CameraCorrection src/NYUNY_CameraCorrection.cpp src/NE_utilities.cpp)
target_link_libraries(NYUNY_CameraCorrection   ${catkin_LIBRARIES})

add_executable(rhombi_logger src/rhombi_logger.cpp src/NE_utilities.cpp)
target_link_libraries(rhombi_logger ${catkin_LIBRARIES})

add_executable(Utku_logger_potential_landing src/Utku_logger_potential_landing.cpp src/NE_utilities.cpp)
target_link_libraries(Utku_logger_potential_landing ${catkin_LIBRARIES})
