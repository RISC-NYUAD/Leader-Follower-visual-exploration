#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

//ARUCO3
#include "aruco.h"
#include <cvdrawingutils.h>
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
bool USE_PREV_RVEC_TVEC; 	//Use previous rvec tvec as initial guess for solvePnP
bool DO_FCU_REFINEMENT;  	//Refine previous rvec tvec with new FCU pose
bool DO_CROP_IMAGE;			//Choose if you want to crop the image for speed
int crop_width, crop_height; //If cropping enabled choose the ROI

int NUM_RHOMBIS;			//Number of Rhombis to expect on this experiment
int self_rhombi;			//# of Rhombi fixed on the drone
int min_detections;			//min # of marker detections per Rhombi to proceed with solvePnP (if this>1 single solution exist) 

int DETECTION_MODE; 		//ArUco detection mode: DM_NORMAL=0,DM_FAST=1,DM_VIDEO_FAST=2
int REFINEMENT_MODE;		//ArUco Corner Refinement: CORNER_SUBPIX=0,CORNER_LINES=1,CORNER_NONE=2
int DICTIONARY;				//ArUco dict to use: ARUCO_MIP_36h12=1, ARUCO=2, ARUCO_MIP_25h7=3, ARUCO_MIP_16h3=4 etc.
std::string image_encoding;	//Options in cv are: "mono8" "bgr8" "bgra8" "rgb8" "rgba8" "mono16" etc.

double contrast;			//Image contrast refinement
double brightness;			//Image brightness refinement

std::string image_savepath; //Image save path
std::string rhombi_gmtry_path; //Filename containing the geometry of Rhombi

struct marker_geom{
	std::vector<std::vector<int>> RhombiMarkerConfigs; //holds the marker ids the form different Rhombis 
	std::vector<std::vector<double>> RhombiCornerGeometries; //holds the marker corners per Rhombi
};  						//Struct holding the geometry of the rhombi

struct rhombi_detect{
	cv::Mat1d	rvec;	//rvecs
	cv::Mat1d	tvec;	//tvecs
};

//cv::Mat cameraIntrincics(3,3,CV_64FC1);
//cv::Mat distortionCoeffs(1,5,CV_64FC1);

int image_width, image_height;

geometry_msgs::PoseArray pose_publishing; //Message for pose_publishing of the detected Rhombis
