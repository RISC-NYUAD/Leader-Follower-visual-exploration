#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Time.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <termios.h>
#include <fcntl.h>

int RATE; 					//ROS rate 
std::string object_name; 	//Robot name
bool DEBUG; 				//Print debug messages on output
bool PREVIEW; 				//Show image preview
bool SAVE_IMAGES; 			//Save images to disk
std::string image_encoding;	//Options in cv are: "mono8" "bgr8" "bgra8" "rgb8" "rgba8" "mono16" etc.

double contrast;			//Image contrast refinement
double brightness;			//Image brightness refinement

std::string image_savepath;		//Image save path
bool experiment_play = false;	//Start the experiment

image_transport::Publisher camera_image_pubs;
sensor_msgs::CameraInfoPtr cam_info_msgs;
std_msgs::Header cam_info_Headers;
