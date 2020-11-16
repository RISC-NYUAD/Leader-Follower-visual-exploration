#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/foreach.hpp>

//ARUCO3
#include "aruco.h"
#include <cvdrawingutils.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace aruco;//ARUCO3
using namespace cv;

#define DEBUG 1
#define PREVIEW 1

#define _DETECTION_MODE aruco::DM_NORMAL 
#define _REFINEMENT_MODE aruco::CORNER_NONE //CORNER_SUBPIX_
#define _DICTIONARY "ARUCO_MIP_16h3" //"ARUCO_MIP_36h12"
#define NUM_RHOMBIS 3 //Number of Rhombis in the experiment

MarkerDetector Mdetector;
std::map<uint32_t, MarkerPoseTracker> Mtracker;
aruco::CameraParameters CamParam;

//ARUCO MARKER SIZE and PARAMS DEFINITION
const double actualRectSideSize = 0.074;
double distSideToSolidCenter;
const double tileSize = 0.0635; //final solid size
const double SmalltileSize = 0.033*tileSize/0.074;
float markerLength=0.01 ;

int frameIDS = 0;

std::string image_encoding="mono8"; //options in cv are: "mono8" "bgr8" "bgra8" "rgb8" "rgba8" "mono16"

//Start ID of marker
vector<int> marker_start_ids={0,17,34}; //if size()=3 means we are looking for 3 Rhombis
//Saves the markerIDS per Rhombi
vector<vector<int>> RhombiConfigs;
//Saves the 3D Geometry per Rhombi
vector<vector<double>> MarkerRhombiGeometries;
//Saves the detected Rhombi ID when ArucoProcess runs
vector<vector<aruco::Marker>> RhombiMarkerDetections(NUM_RHOMBIS);

vector<vector<cv::Mat>> RvecTvec(NUM_RHOMBIS);

// Calculates rotation matrix to euler angles
Vec3f rotationMatrixToEulerAngles(Mat &R)
{      
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) ); 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);     
}

bool inRange(int low,  int high,  int x)
{
return (low < x && x < high);
}

double compute_distance(cv::Mat1d tvec_prev, cv::Mat1d tvec_new ){
	return sqrt(pow(tvec_prev(0,0)-tvec_new(0,0),2)+pow(tvec_prev(0,1)-tvec_new(0,1),2)+pow(tvec_prev(0,2)-tvec_new(0,2),2));
}


bool separate_markersNE(vector<aruco::Marker> Markers){
	bool found=false;
	std::vector<int> rhombi_exists(NUM_RHOMBIS,0);
	for(auto& marker : Markers){
		int index = marker.id/17;
		if(index<=NUM_RHOMBIS){
			RhombiMarkerDetections[index].push_back(marker);
			found=true;
		}
	}
	return found;
}

void arucoProcessNE(cv::Mat srcImg){
	//Copy Image for visualizing
	cv::Mat imageMarkers;
	srcImg.copyTo(imageMarkers);
	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat imageAxes(srcImg.size(), CV_8UC3);
	cv::cvtColor(srcImg, imageAxes, CV_GRAY2RGB);
	//cv::Mat imageAxes;
	//srcImg.copyTo(imageAxes);
	
	vector<Marker> Markers = Mdetector.detect(srcImg); // find all markers in frame
	if(Markers.size()>0){
		if(PREVIEW){
			for(auto m : Markers) {
				m.draw(imageMarkers, Scalar(0, 0, 255), 2);
				imshow("imageMarkers",imageMarkers); 
				//cv::waitKey(1);
			}
		}
		if(separate_markersNE(Markers)){ // do nothing if No Rhombis were found
			for (int RhombiIndex=0; RhombiIndex< RhombiMarkerDetections.size(); RhombiIndex++){ // for every different rhombi seen (rows)
				vector<aruco::Marker> Rhombi=RhombiMarkerDetections[RhombiIndex];
				//Check if more than one marker we can run cv::solvePnp
				if(Rhombi.size()>0){
					vector<cv::Point3d>  objectPoints;
					vector<cv::Point2d>  imagePoints;
					//Assign 3D points and 2D image points for solving
					for(int j=0;j<Rhombi.size();j++){ 
						//for every different Marker seen (cols) first ID is in Col 1
						aruco::Marker marker=Rhombi[j];
						//Look up marker placement in the Rhombi  -- Note that although in different Rhombis IDs 0 and 18 have the same geometrical placement, thus index truncated to Rhombi 1
						int index = marker.id-17*(marker.id/17);
						if(DEBUG) {std::cout << "Iter: " << j << " Marker ID: " << marker.id << " index: " << index << " and points: " <<std::endl;}
						for(int c=0;c<4;c++){
							//and copy coordinates and 2D image point to solver
							objectPoints.push_back(Point3d(MarkerRhombiGeometries[index*4+c][0],MarkerRhombiGeometries[index*4+c][1],MarkerRhombiGeometries[index*4+c][2]));
							imagePoints.push_back(marker[c]);
							if(DEBUG) {
								std::cout << MarkerRhombiGeometries[index*4+c][0] << " " << MarkerRhombiGeometries[index*4+c][1] << " " << 
																	MarkerRhombiGeometries[index*4+c][2] << std::endl;
								std::cout << marker[c] << std::endl;
							}
						}
					}
					//if previous values exist pass them to solve SolvePNP
					cv::Mat error;
					cv::Mat rvec_det,tvec_det;
					if(RvecTvec[RhombiIndex].size()>0){
						//Save previous values
						cv::Mat rvec_prev= rvec_det = RvecTvec[RhombiIndex][0];
						cv::Mat tvec_prev= tvec_det = RvecTvec[RhombiIndex][1];
						cv::solvePnP(objectPoints, imagePoints, CamParam.CameraMatrix, CamParam.Distorsion, rvec_det, tvec_det, true, 0); 
						//cv::solvePnPGeneric(objectPoints,imagePoints,CamParam.CameraMatrix, CamParam.Distorsion,rvecs_det,tvecs_det, true, 0, RvecTvec[RhombiIndex][0], RvecTvec[RhombiIndex][1], error); 
						//Clear anyway...if eroneous value we should compute from scrath
						RvecTvec[RhombiIndex].empty();
						//Check for eroneous values in translation -- if not solved keep previous values for next iteration
						if(!rvec_det.empty() && compute_distance(tvec_det, tvec_prev) < 1.0){
							RvecTvec[RhombiIndex].push_back(rvec_det);
							RvecTvec[RhombiIndex].push_back(tvec_det);
						}
					}
					else{
						cv::solvePnP(objectPoints, imagePoints, CamParam.CameraMatrix, CamParam.Distorsion, rvec_det, tvec_det, false, 3);
						if(!rvec_det.empty()){
							RvecTvec[RhombiIndex].push_back(rvec_det);
							RvecTvec[RhombiIndex].push_back(tvec_det);
						}
					}
					if(!rvec_det.empty()) {
						cv::Mat to_matrix;
						cv::Vec3f to_euler_xyz;
						cv::Rodrigues(rvec_det, to_matrix);
						to_euler_xyz=rotationMatrixToEulerAngles(to_matrix);
						std::cout << "Rhombi r_vec: " << to_euler_xyz[0]*180/M_PI << " " << to_euler_xyz[1]*180/M_PI << " " << to_euler_xyz[2]*180/M_PI << " " << std::endl;
						std::cout << "Rhombi t_vec: " << tvec_det << std::endl << std::endl;;}
					if(PREVIEW && !rvec_det.empty()){
						cv::Mat to_matrix;
						cv::Vec3f to_euler_xyz;
						cv::Rodrigues(rvec_det, to_matrix);
						to_euler_xyz=rotationMatrixToEulerAngles(to_matrix);
						vector< Point2f > imagePointsA;	
						vector< Point3f > axisPoints;
						axisPoints.push_back(Point3f(0, 0, 0));
						axisPoints.push_back(Point3f(markerLength * 7.5f, 0, 0));
						axisPoints.push_back(Point3f(0, markerLength * 7.5f, 0));
						axisPoints.push_back(Point3f(0, 0, markerLength * 7.5f));			
						Mat imagePointsVelocities = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); 
						projectPoints(axisPoints, rvec_det, tvec_det, CamParam.CameraMatrix, CamParam.Distorsion, imagePointsA, imagePointsVelocities);
						// draw axis lines
						int C_w=CamParam.CamSize.width;
						int C_h=CamParam.CamSize.height;
						if(inRange(0,C_w,imagePointsA[0].x) && inRange(0,C_h,imagePointsA[0].y) &&
							inRange(0,C_w,imagePointsA[1].x) && inRange(0,C_h,imagePointsA[1].y) &&
								inRange(0,C_w,imagePointsA[2].x) && inRange(0,C_h,imagePointsA[2].y) &&
									inRange(0,C_w,imagePointsA[3].x) && inRange(0,C_h,imagePointsA[3].y)){
							line(imageAxes, imagePointsA[0], imagePointsA[1], Scalar(0, 0, 255), 10);
							line(imageAxes, imagePointsA[0], imagePointsA[2], Scalar(0, 255, 0), 10);
							line(imageAxes, imagePointsA[0], imagePointsA[3], Scalar(255, 0, 0), 10);
						}
					}
					else{
						if(DEBUG) ROS_INFO("No solution found for Rhombi.");
					}
				}
			}
			if(PREVIEW){imshow("RhombiAxes",imageAxes);cv::waitKey(1);}
		}
		else{
			if(DEBUG) ROS_INFO("Wrong marker seperation");
		}
		//Clear previous detections
		for(int i=0 ; i< RhombiMarkerDetections.size(); i++)
		RhombiMarkerDetections[i].clear();
	}
	else{
		if(DEBUG) ROS_INFO("No markers found");
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) //(const sensor_msgs::ImageConstPtr& msg, int image360part)
{
  try
  {
	cv::Mat srcImg = cv_bridge::toCvShare(msg, image_encoding)->image;
	if(!srcImg.empty()){
		frameIDS++;
		arucoProcessNE(srcImg);
	}//! empty check

  }
  catch (cv_bridge::Exception& e)
  {
 	//ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	ROS_ERROR("Could not convert from %s",image_encoding.c_str());
  }
}

//Init Aruco3
void aruco3_init(){
	//Generate Rhombis
	vector<int> ids;
	for(auto i : marker_start_ids){
		//Push back rectangle surfaces - final 4 ids are from triangular markers
		for(int j=0; j<17; j++) ids.push_back(i+j);
		RhombiConfigs.push_back(ids);
		ids.clear();
	}
	//Generate geometries -- They are the same for every Rhombi
	distSideToSolidCenter = ((sqrt(2.0f)+1.0f)/2.0f) * actualRectSideSize;
	std::vector<double> corner_final;
	//A)Lower big markers
	for(int j=0; j<8 ; j++){
		//1) FIND CENTER OF MARKER
		if(DEBUG) std::cout << "Marker No. " << j << " " << std::endl;
		//Create RT matrix of Marker
		Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Rotate First Eigen uses angle-axis rotation system - rotation for lower markers is over Y axis
		Eigen::AngleAxisd MarkerRotation_vector ( M_PI_4*j, Eigen::Vector3d ( 0, 1 , 0 ) );
		MarkerRT.rotate(MarkerRotation_vector);
		//Translate second -- Lower markers are higher in Y (up) axis by actualRectSideSize/2 and in Z (forward) by actualRectSideSize
		Eigen::Vector3d MarkerTrans(0, actualRectSideSize/2, distSideToSolidCenter);
		MarkerRT.translate(MarkerTrans);
		
		//2) Create RT matrix that will get us to CORNERS
		Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Corners -- For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
		Eigen::Vector3d CornerTrans(0, tileSize*sqrt(2)/2, 0);
		Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
		CornersRT_base.rotate(CornerRots);
		
		//3) Multiply RTs to get final corners and assign
		for(int k=0;k<4; k++){
			Eigen::Isometry3d CornersRT=CornersRT_base;
			Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT.rotate(CornerRotation_vector);
			CornersRT.translate(CornerTrans);
			Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
			//std::cout << "Corner_Final " << std::endl;
			for(int c=0;c<3;c++) {corner_final.push_back(_Corner_Final(c,3)); if(DEBUG) std::cout << _Corner_Final(c,3) << " " ;}
			if(DEBUG) std::cout << std::endl;
			MarkerRhombiGeometries.push_back(corner_final);
			corner_final.clear();
		}
		if(DEBUG) std::cout << std::endl;
	}
	//B)Middle big markers
	for(int j=0; j<4 ; j++){
		//1) FIND CENTER OF MARKER
		if(DEBUG)  std::cout << "Marker No. " << j+8 << " " << std::endl;
		//Create RT matrix of Marker
		Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Translate first 
		Eigen::Vector3d MarkerTrans(0, actualRectSideSize/2, 0);
		MarkerRT.translate(MarkerTrans);
		//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
		Eigen::AngleAxisd MarkerRotation_vector ( M_PI_2*j, Eigen::Vector3d ( 0, 1 , 0 ) );
		MarkerRT.rotate(MarkerRotation_vector);
		//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
		Eigen::AngleAxisd MarkerRotation_vector2 ( -M_PI_4, Eigen::Vector3d ( 1, 0 , 0 ) );
		MarkerRT.rotate(MarkerRotation_vector2);
		//Final translate over new Z
		Eigen::Vector3d MarkerTrans2(0, 0, distSideToSolidCenter);
		MarkerRT.translate(MarkerTrans2);

		
		//2) Create RT matrix that will get us to CORNERS
		Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Corners -- For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
		Eigen::Vector3d CornerTrans(0, tileSize*sqrt(2)/2, 0);
		Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
		CornersRT_base.rotate(CornerRots);
		
		//3) Multiply RTs to get final corners and assign
		for(int k=0;k<4; k++){
			Eigen::Isometry3d CornersRT=CornersRT_base;
			Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT.rotate(CornerRotation_vector);
			CornersRT.translate(CornerTrans);
			Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
			//std::cout << "Corner_Final " << std::endl;
			for(int c=0;c<3;c++) {corner_final.push_back(_Corner_Final(c,3)); if(DEBUG)  std::cout << _Corner_Final(c,3) << " " ;}
			if(DEBUG) std::cout << std::endl;
			MarkerRhombiGeometries.push_back(corner_final);
			corner_final.clear();
		}
		if(DEBUG)  std::cout << std::endl;
	}
	//C)Upper marker
	for(int j=0; j<1 ; j++){
		//1) FIND CENTER OF MARKER
		if(DEBUG)  std::cout << "Marker No. " << j+12 << " " << std::endl;
		//Create RT matrix of Marker
		Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Translate first 
		Eigen::Vector3d MarkerTrans(0, actualRectSideSize/2, 0);
		MarkerRT.translate(MarkerTrans);
		//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
		Eigen::AngleAxisd MarkerRotation_vector ( M_PI_2*j, Eigen::Vector3d ( 0, 1 , 0 ) );
		MarkerRT.rotate(MarkerRotation_vector);
		//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
		Eigen::AngleAxisd MarkerRotation_vector2 ( -M_PI_2, Eigen::Vector3d ( 1, 0 , 0 ) );
		MarkerRT.rotate(MarkerRotation_vector2);
		//Final translate over new Z
		Eigen::Vector3d MarkerTrans2(0, 0, distSideToSolidCenter);
		MarkerRT.translate(MarkerTrans2);

		
		//2) Create RT matrix that will get us to CORNERS
		Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Corners -- For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
		Eigen::Vector3d CornerTrans(0, tileSize*sqrt(2)/2, 0);
		Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
		CornersRT_base.rotate(CornerRots);
		
		//3) Multiply RTs to get final corners and assign
		for(int k=0;k<4; k++){
			Eigen::Isometry3d CornersRT=CornersRT_base;
			Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT.rotate(CornerRotation_vector);
			CornersRT.translate(CornerTrans);
			Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
			//std::cout << "Corner_Final " << std::endl;
			for(int c=0;c<3;c++) {corner_final.push_back(_Corner_Final(c,3)); if(DEBUG) std::cout << _Corner_Final(c,3) << " " ;}
			if(DEBUG) std::cout << std::endl;
			MarkerRhombiGeometries.push_back(corner_final);
			corner_final.clear();
		}
		if(DEBUG) std::cout << std::endl;
	}
	//D)Smaller markers
	for(int j=0; j<4 ; j++){
		//1) FIND CENTER OF MARKER
		if(DEBUG) std::cout << "Marker No. " << j+13 << " " << std::endl;
		//Create RT matrix of Marker
		Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Translate first 
		Eigen::Vector3d MarkerTrans(0, actualRectSideSize/2, 0);
		MarkerRT.translate(MarkerTrans);
		//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
		Eigen::AngleAxisd MarkerRotation_vector ( M_PI_4+M_PI_2*j, Eigen::Vector3d ( 0, 1 , 0 ) );
		MarkerRT.rotate(MarkerRotation_vector);
		//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
		Eigen::AngleAxisd MarkerRotation_vector2 ( -M_PI_4, Eigen::Vector3d ( 1, 0 , 0 ) );
		MarkerRT.rotate(MarkerRotation_vector2);
		//Final translate over new Z
		Eigen::Vector3d MarkerTrans2(0, -SmalltileSize/2, distSideToSolidCenter);
		MarkerRT.translate(MarkerTrans2);
		//Final translate over new point to go exactly to triangle center 
		
		//2) Create RT matrix that will get us to CORNERS
		Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
		//Corners -- For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
		Eigen::Vector3d CornerTrans(0, SmalltileSize*sqrt(2)/2, 0);
		Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
		CornersRT_base.rotate(CornerRots);
		
		//3) Multiply RTs to get final corners and assign
		for(int k=0;k<4; k++){
			Eigen::Isometry3d CornersRT=CornersRT_base;
			Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT.rotate(CornerRotation_vector);
			CornersRT.translate(CornerTrans);
			Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
			for(int c=0;c<3;c++) {corner_final.push_back(_Corner_Final(c,3)); if(DEBUG) std::cout << _Corner_Final(c,3) << " " ;}
			if(DEBUG) std::cout << std::endl;
			MarkerRhombiGeometries.push_back(corner_final);
			corner_final.clear();
		}
		if(DEBUG) std::cout << std::endl;
	}
	if(DEBUG){ for(int k=0; k<MarkerRhombiGeometries.size(); k++) if(DEBUG) std::cout<< MarkerRhombiGeometries[k][0] << ' ' << MarkerRhombiGeometries[k][1] << ' ' << MarkerRhombiGeometries[k][2] << ' ' << std::endl;}
	//Set detection params
	Mdetector.setDictionary(_DICTIONARY, 0.f);
	Mdetector.setDetectionMode(_DETECTION_MODE,0.f);
	MarkerDetector::Params _param=Mdetector.getParameters();
	_param.setCornerRefinementMethod(_REFINEMENT_MODE);
	Mdetector.setParameters(_param);
}

//MAIN
int main(int argc, char **argv)
{  
	ros::init(argc, argv, "aruco_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	//Init ArUco -- Generate geometries
	aruco3_init();
	
	//Create subscriber to get Camera Info and calibration -- Runs only once
	sensor_msgs::CameraInfoConstPtr _CameraInfo= ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_array/cam0/camera_info", nh);
	if(_CameraInfo != NULL){
		CamParam.setParams(cv::Mat(3, 3, CV_64F, (void *) _CameraInfo->K.data()), cv::Mat(1, 5, CV_64F, (void *) _CameraInfo->D.data()), Size(_CameraInfo->width, _CameraInfo->height));
	}
	else{
		ROS_INFO("Camera info file received is empty. Exiting...");
		return -1;
	}
	
	//Init acquisition
	image_transport::Subscriber sub = it.subscribe("/camera_array/cam0/image_raw", 1, imageCallback);

	//LOOP START
	ros::Rate loop_rate= 10; //Hz
	while(ros::ok() && true){
		ros::spinOnce();
		loop_rate.sleep();
	}
}
