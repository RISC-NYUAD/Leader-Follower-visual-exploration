#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Time.h>

//ARUCO3
#include "aruco.h"
#include "cvdrawingutils.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <iterator> //to calculate distance from beggining in std::vectors

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

std::string image_savepath; //Image save path
std::string rhombi_gmtry_path; //Filename containing the geometry of Rhombi

std::string cam_param_path;
std::string mmap_path;

aruco::MarkerDetector MDetector;
aruco::CameraParameters CamParam;
aruco::MarkerMapPoseTracker MMTracker;

geometry_msgs::PoseStamped MarkerMapPose; //Dummy pose to hold the image frame and time taken
ros::Publisher PosePub;

