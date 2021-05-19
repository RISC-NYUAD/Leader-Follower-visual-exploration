cmake_minimum_required(VERSION 2.8)
project(rhombi_markers)

# require C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package( Eigen3 REQUIRED )
MESSAGE(STATUS "Eigen3 found, version: ${EIGEN3_VERSION} in dir ${EIGEN3_INCLUDE_DIR}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  geometry_msgs
  std_msgs
  tf_conversions
  eigen_conversions
  tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} 
    ${aruco_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
)

#SET(aruco_LIB_DIR "/home/odroid/github_packages/aruco-3.1.12/src"; ${aruco_LIB_DIR} )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslaunch
    cv_bridge
    image_transport
    camera_info_manager
    sensor_msgs
    geometry_msgs
    std_msgs
    tf_conversions
    eigen_conversions
    tf
)

#SET(OpenCV_DIR ~/github_packages/opencv_3_4_4/opencv/build/)
find_package(OpenCV 3.2 REQUIRED)
#find_package(OpenCV 3.4 REQUIRED PATHS "~/github_packages/opencv/build")
#MESSAGE(STATUS "OpenCV found!!!!!!!!!!!!!!!!!!!!!!!!, version: ${OpenCV_VERSION} in dir ${OpenCV_INCLUDE_DIRS}")

SET(aruco_DIR ~/github_packages/aruco-3.1.12/build)
find_package(aruco REQUIRED )
MESSAGE(STATUS "aruco !!!!!!!!!!!!!!!!! found, version: ${aruco_VERSION} in dir ${aruco_INCLUDE_DIRS}")

#MESSAGE("1")
#MESSAGE(STATUS ${OpenCV_LIB_DIRS} )
#MESSAGE("2")

#add_executable(aruco_simple src/aruco_simple.cpp)
#target_link_libraries(aruco_simple ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

#add_executable(aruco_findRcube2 src/aruco_findRcube_v2.cpp)
#target_link_libraries(aruco_findRcube2 ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

#add_executable(aruco_findRcubeNE src/aruco_findRcube_v3.cpp)
#target_link_libraries(aruco_findRcubeNE ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(rhombi_markers src/rhombi_markers.cpp)
target_link_libraries(rhombi_markers ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(rhombi_markers_v2 src/rhombi_markers_v2.cpp)
target_link_libraries(rhombi_markers_v2 ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(rhombi_detect src/rhombi_detect.cpp)
target_link_libraries(rhombi_detect ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(NYUNY_experiment src/NYUNY_experiment.cpp)
target_link_libraries(NYUNY_experiment ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(rhombi_geom_extractor src/rhombi_geom_extractor.cpp)
target_link_libraries(rhombi_geom_extractor ${aruco_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES} -lstdc++fs)
