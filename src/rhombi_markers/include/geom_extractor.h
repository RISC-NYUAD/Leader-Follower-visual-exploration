#include <ros/ros.h>

#include <math.h>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sstream>
#include <fstream>
#include <iostream>
#include <experimental/filesystem>

double _RectangleSide;		//Size of rhombicuboctahdron side
double _MarkerSize;			//Size of big ArUco marker side
double _MarkerSmallSize;	//Size of small ArUco marker side

std::string image_savepath;
std::string geometry_name;

void aruco3_init(double , double , double, std::string );
