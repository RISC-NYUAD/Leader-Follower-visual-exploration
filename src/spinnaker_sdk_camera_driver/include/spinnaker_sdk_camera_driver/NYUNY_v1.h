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
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

//ARUCO3
#include "aruco.h"
#include "cvdrawingutils.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

//For cv::Rodrigues and cv::cv2eigen
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

//Spinnaker
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <chrono>

//NE
#include "spinnaker_sdk_camera_driver/NE_utilities.h"

int RATE; 					//ROS rate 
std::string object_name; 	//Robot name
bool DEBUG; 				//Print debug messages on output
bool PREVIEW; 				//Show image preview
bool SAVE_IMAGES; 			//Save images to disk

int DETECTION_MODE; 		//ArUco detection mode: DM_NORMAL=0,DM_FAST=1,DM_VIDEO_FAST=2
int REFINEMENT_MODE;		//ArUco Corner Refinement: CORNER_SUBPIX=0,CORNER_LINES=1,CORNER_NONE=2
int DICTIONARY;				//ArUco dict to use: ARUCO_MIP_36h12=1, ARUCO=2, ARUCO_MIP_25h7=3, ARUCO_MIP_16h3=4 etc.
std::string image_encoding;	//Options in cv are: "mono8" "bgr8" "bgra8" "rgb8" "rgba8" "mono16" etc.

double contrast;			//Image contrast refinement
double brightness;			//Image brightness refinement

std::string image_savepath;		//Image save path
bool experiment_play = false;	//Start the experiment

std::string cam_param_path;		//Path holding the camera parameters
std::string mmap_path;			//String holding the marker map configuration in meters

aruco::MarkerDetector MDetector;	//ArUco marker detector
aruco::CameraParameters CamParam;	//ArUco final camera parameters file
aruco::MarkerMapPoseTracker MMTracker;	//Tracker holding the marker map to detect

image_transport::Publisher camera_image_pubs; 	//The image
std_msgs::Header cam_info_Headers;				//The image header

ros::Publisher PosePub;						//Publisher for sending the pose only!
geometry_msgs::PoseStamped MarkerMapPose; 	//Message to hold the ArUco pose
geometry_msgs::PoseStamped Vicon_MonitorPose, Vicon_CCDPose; //Message to hold Vicon Values

bool image_to_ros;				//Choose to publish image to ros

void log_measurements(void);
