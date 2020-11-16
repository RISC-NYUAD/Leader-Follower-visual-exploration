#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/foreach.hpp>
 
int camWidth = 2048; 
int camHeight = 1536; 

//./ocamsC -g=1 -h=7 -l=0.0100 -s=0.0150 -w=5 -ci=3 --rs -d=10 -dp="detector_params.yml" -c="outCameraCalib78.txt"
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>

#include <string> 
#include <math.h>
#include <fstream>

//ARDUINO COM READ
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#define DEBUG 1
#include <thread>
//#include <string.h>
//#include <termios.h>
//#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp> 
#include <opencv2/calib3d.hpp> //add Rodrigues transformation

//ARUCO3
#include "aruco.h"
#include "cvdrawingutils.h"
#include <cvdrawingutils.h>

using namespace std;
using namespace cv;
using namespace aruco;//ARUCO3

vector< Mat > Final_Results; 
MarkerDetector Mdetector;
std::map<uint32_t, MarkerPoseTracker> Mtracker;
aruco::CameraParameters CamParam;

double PI = 3.141592653589793;

std::string image_encoding="mono8"; //options in cv are: "mono8" "bgr8" "bgra8" "rgb8" "rgba8" "mono16"

typedef struct MarkerOrientation{ // struct to return x,y angles
	double x_rot;
	double y_rot;
}XY_rot;

typedef struct SeparatedMarkers{ // struct to return list of separated markers per rhombi and their rhombi identifier
	vector< vector< Marker > > markers_list;
	vector<int> flags;
}Markers_List;

Markers_List separate_markers(vector<aruco::Marker> Markers){
	vector<vector<aruco::Marker>> rhombi_markers;
	vector<vector<aruco::Marker>> temp;
	vector<aruco::Marker> rhombi_1;
	vector<aruco::Marker> rhombi_2;
	vector<aruco::Marker> rhombi_3;
	vector<int> counters(3, 0); // initialize a 3-element zero vector
	vector<int> flags;
	for(auto& marker : Markers){
		int num = marker.id;
		int index = floor(num/53.)+1;
		switch(index){
			case(1):
				counters.at(0)++;
				rhombi_1.push_back(marker);
				break;
			case(2):
				counters.at(1)++;
				rhombi_2.push_back(marker);
				break;
			case(3):
				counters.at(2)++;
				rhombi_3.push_back(marker);
				break;
		}
	}
	temp.push_back(rhombi_1);
	temp.push_back(rhombi_2);
	temp.push_back(rhombi_3); // temp is a holder, because it is certain that at least some of them are empty, we will only send forward non-empty ones
	for(int j = 0; j<3 ; j++){
		if(counters[j]>0){
			rhombi_markers.push_back(temp[j]);
			flags.push_back(j);
		}
	}
	// so now, rhombi_markers has vectors of markers of specific rhombicubes
	// and flags has integers specifying the rhombi of each rhombi_markers vector
	Markers_List list;
	list.markers_list = rhombi_markers;
	list.flags = flags;
	return list;
}

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =	
	"{HorAngle |       | Measured horizontal angle }"
	"{VerAngle |       | Measured vertical angle }"
	"{RealDist |       | Measured distance from solid center }"
	"{af       |       | 0 for no preview, 1 for preview window }"
	"{shape    |       | 0 for 21edron, 1 for Cube }"
	"{p        |       | 0 for no preview, 1 for preview window }"
	"{usePOZYX |       | 0 no POZYX, 1 enable POZYX}"
	"{useLIDAR |       | 0 no LIDAR, 1 enable LIDAR}"
	"{PCDcloud |       | Point cloud for reference 3D object}"
	"{g        |       | 0 for color, 1 for grey }"
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side lenght (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

//// CAMERA

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}
/**
 */


//MATRIX FUNCTIONS 3 x 1
Vec3d matrix31_abs(Vec3d inputmatrix){
	inputmatrix(0) = abs(inputmatrix(0));
	inputmatrix(1) = abs(inputmatrix(1));
	inputmatrix(2) = abs(inputmatrix(2));
	return inputmatrix;
}
void zero31_matrix(Vec3d &rotResultC){
	rotResultC(0) = 0;
	rotResultC(1) = 0;
	rotResultC(2) = 0;	
}
void matrix32_assign_from_to(Vec3d assignFrom, Vec3d &assignTo){
	assignTo(0) = assignFrom(0);
	assignTo(1) = assignFrom(1);
	assignTo(2) = assignFrom(2);	
}
void add_to_31_matrix(Vec3d &rotResultCTMP, Vec3d add_this){
	rotResultCTMP(0) = rotResultCTMP(0) + add_this(0);
	rotResultCTMP(1) = rotResultCTMP(1) + add_this(1);
	rotResultCTMP(2) = rotResultCTMP(2) + add_this(2);		
}
void scale31_matrix(Vec3d &rotResultCTMP, float divider)
{
	rotResultCTMP(0) = rotResultCTMP(0) / divider;
	rotResultCTMP(1) = rotResultCTMP(1) / divider;
	rotResultCTMP(2) = rotResultCTMP(2) / divider;	
}
float matrix31_distance(Vec3d centerRotationsTMP, Vec3d centerRotationsTMP2){
	float center_diffX1 = centerRotationsTMP(0) - centerRotationsTMP2(0);		
	float center_diffX2 = centerRotationsTMP(1) - centerRotationsTMP2(1);		
	float center_diffX3 = centerRotationsTMP(2) - centerRotationsTMP2(2);	
	float Center_to_Prev_Center_dist = sqrt(pow(center_diffX1,2) + pow(center_diffX2,2) + pow(center_diffX3,2));
	return Center_to_Prev_Center_dist;
}
//MATRIX FUNCTIONS 3 x 3
void zero33_matrix(Mat &rotResultC){
	rotResultC.at<double>(0,0) = 0;
	rotResultC.at<double>(0,1) = 0;
	rotResultC.at<double>(0,2) = 0;
	rotResultC.at<double>(1,0) = 0;
	rotResultC.at<double>(1,1) = 0;
	rotResultC.at<double>(1,2) = 0;
	rotResultC.at<double>(2,0) = 0;
	rotResultC.at<double>(2,1) = 0;
	rotResultC.at<double>(2,2) = 0;
}
void add_to_33_matrix(Mat &rotResultCTMP, Mat add_this){
	rotResultCTMP.at<double>(0,0) = rotResultCTMP.at<double>(0,0) + add_this.at<double>(0,0);
	rotResultCTMP.at<double>(0,1) = rotResultCTMP.at<double>(0,1) + add_this.at<double>(0,1);
	rotResultCTMP.at<double>(0,2) = rotResultCTMP.at<double>(0,2) + add_this.at<double>(0,2);
	rotResultCTMP.at<double>(1,0) = rotResultCTMP.at<double>(1,0) + add_this.at<double>(1,0);
	rotResultCTMP.at<double>(1,1) = rotResultCTMP.at<double>(1,1) + add_this.at<double>(1,1);
	rotResultCTMP.at<double>(1,2) = rotResultCTMP.at<double>(1,2) + add_this.at<double>(1,2);
	rotResultCTMP.at<double>(2,0) = rotResultCTMP.at<double>(2,0) + add_this.at<double>(2,0);
	rotResultCTMP.at<double>(2,1) = rotResultCTMP.at<double>(2,1) + add_this.at<double>(2,1);
	rotResultCTMP.at<double>(2,2) = rotResultCTMP.at<double>(2,2) + add_this.at<double>(2,2);
	//return rotResultCTMP;
}
void scale33_matrix(Mat &rotResultCTMP, float divider)
{
	rotResultCTMP.at<double>(0,0) = rotResultCTMP.at<double>(0,0) / divider;
	rotResultCTMP.at<double>(0,1) = rotResultCTMP.at<double>(0,1) / divider;
	rotResultCTMP.at<double>(0,2) = rotResultCTMP.at<double>(0,2) / divider;
	rotResultCTMP.at<double>(1,0) = rotResultCTMP.at<double>(1,0) / divider;
	rotResultCTMP.at<double>(1,1) = rotResultCTMP.at<double>(1,1) / divider;
	rotResultCTMP.at<double>(1,2) = rotResultCTMP.at<double>(1,2) / divider;
	rotResultCTMP.at<double>(2,0) = rotResultCTMP.at<double>(2,0) / divider;
	rotResultCTMP.at<double>(2,1) = rotResultCTMP.at<double>(2,1) / divider;
	rotResultCTMP.at<double>(2,2) = rotResultCTMP.at<double>(2,2) / divider;
}
float matrix33_distance(Mat centerRotationsTMP, Mat centerRotationsTMP2){
	float center_diffX1 = centerRotationsTMP.at<double>(0,0) - centerRotationsTMP2.at<double>(0,0);		
	float center_diffX2 = centerRotationsTMP.at<double>(0,1) - centerRotationsTMP2.at<double>(0,1);		
	float center_diffX3 = centerRotationsTMP.at<double>(0,2) - centerRotationsTMP2.at<double>(0,2);		
	float center_diffY1 = centerRotationsTMP.at<double>(1,0) - centerRotationsTMP2.at<double>(1,0);		
	float center_diffY2 = centerRotationsTMP.at<double>(1,1) - centerRotationsTMP2.at<double>(1,1);		
	float center_diffY3 = centerRotationsTMP.at<double>(1,2) - centerRotationsTMP2.at<double>(1,2);	
	float center_diffZ1 = centerRotationsTMP.at<double>(2,0) - centerRotationsTMP2.at<double>(2,0);		
	float center_diffZ2 = centerRotationsTMP.at<double>(2,1) - centerRotationsTMP2.at<double>(2,1);		
	float center_diffZ3 = centerRotationsTMP.at<double>(2,2) - centerRotationsTMP2.at<double>(2,2);
	float Center_to_Prev_Center_dist = sqrt(pow(center_diffX1,2) + pow(center_diffX2,2) + pow(center_diffX3,2) 
	+ pow(center_diffY1,2) + pow(center_diffY2,2) + pow(center_diffY3,2) 
	+ pow(center_diffZ1,2) + pow(center_diffZ2,2) + pow(center_diffZ3,2));
	return Center_to_Prev_Center_dist;
}
Mat rotation_matrix_Z_rad(float rad){
 	return (Mat_<double>(4,4) << 
	cos(rad),-sin(rad),0,0, 
	sin(rad),cos(rad),0,
	0,0,0,1,0, 
	0,0,0,1);
}
Mat rotation_matrix_Y_rad(float rad){
 	return (Mat_<double>(4,4) << 
	cos(rad),0,sin(rad),0,   
	0,1,0,0,   
	-sin(rad),0,cos(rad),0,    
	0,0,0,1);
}
Mat rotation_matrix_X_rad(float rad){
 	return (Mat_<double>(4,4) << 
	1,0,0,0,  
	0,cos(rad),-sin(rad),0,   
	0,sin(rad),cos(rad),0, 
	0,0,0,1);
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
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



//// COMBINATIONS
//METHOD 1 //find combinations and choose the group with least members (as it will not contain the offending outlier)		
vector<int> outlierComboIDs;
int comboIteration;
vector<int> foundIDs;
vector<int> combination;
vector<vector<int>> combinations;
int errorID = -1;
int comboDepthLevel=0;

void generateCobminations(int offset, int k) {
	if (k == 0) {			    
		combinations.push_back(combination);
		return;
	}
	for (int i = offset; i <= foundIDs.size() - k; ++i) {
		combination.push_back(foundIDs[i]);
		generateCobminations(i+1, k-1);
		combination.pop_back();
	}
}

void discoverOutliersMinMax( vector<cv::Vec<double, 3>> &centerPointsTMP, vector<int> &centerPointsIDsTMP, std::vector<cv::Mat> &centerRotationsTMP ){

				comboIteration += 0.01; //go to 1 in first round
				comboDepthLevel++; // increase depth
				int outofThresCount = 0; //count how many out of threshold

				//find combinations and choose the group with least members (as it will not contain the offending outlier)				
				float distThreshold = 0.04f; //0.001f;//0.015 - comboIteration; //0.014f; //5cm threshold to determine clusters //CHANGE HERE
				int nCombinations = centerPointsIDsTMP.size(); //ids.size(); //centerPointsIDs , centerPoints
				int kCombinations = nCombinations-1; //start with max, reduce up to 2

				foundIDs.clear();//reset ids with new ones
				combinations.clear();
				combination.clear();

				int biggestDistID = 0;
				int smallerDistID = 0;
				float currentBiggestDist = 0.0f; //initialize with a threshold
				float currentSmallerDist = 100000; ////CHANGE HERE
				for (int i = 0; i < nCombinations; ++i) 
				{ 
					foundIDs.push_back(i); //foundIDs.push_back(ids[i]); 
				}
				generateCobminations(0, kCombinations);

				//cout << "ids = " << foundIDs.size() << endl; 
				//cout << " Combinations = " << combinations.size() << endl; 

				//ROTATIONS				
				//cout << "ROTATIONS SIZE = " << centerRotationsTMP.size() << endl; 

				if(centerRotationsTMP.size() < 3){ return; }							

				for (int i = 0; i < combinations.size(); i++) { //for (int i = 0; i < combinations.size()-1; i++) {
					//find center average
					Vec3d center1;
					Vec3d center1ROT;
					for (int i1 = 0; i1 < combinations[i].size(); i1++) {
						add_to_31_matrix(center1,centerPointsTMP[combinations[i][i1]]);						
						
						Vec3d toEuler = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[i][i1]])*(180.0f/PI);	
						//cv::Mat R;
						//cv::Rodrigues(centerRotationsTMP[combinations[i][i1]],R);

						add_to_31_matrix(center1ROT, matrix31_abs(toEuler));						
						//cout << "toEuler Angles = " << toEuler << endl; 
					}
					scale31_matrix(center1, combinations[i].size());					
					scale31_matrix(center1ROT, combinations[i].size());					
					//cout << "toEuler Angles CENTER = " << center1ROT << endl; 
				
					float currentDist = 100000000;	//find distance to all others
					for (int j = 0; j < combinations.size(); j++) { //for (int j = i+1; j < combinations.size(); j++) {
						//find center average
						Vec3d center2;
						Vec3d center2ROT;
						for (int j1 = 0; j1 < combinations[j].size(); j1++) {
							add_to_31_matrix(center2,centerPointsTMP[combinations[j][j1]]);
							Vec3d toEuler1 = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[j][j1]])*(180.0f/PI);
							add_to_31_matrix(center2ROT, matrix31_abs(toEuler1));								
						}
						scale31_matrix(center2, combinations[j].size());						
						scale31_matrix(center2ROT, combinations[j].size());						

						float Center_to_Cam_distA = matrix31_distance(center1ROT,center2ROT);							
						
						//cout << "Center_to_Cam_distA AAAAAA= " << Center_to_Cam_distA << endl; 
						
						//MIN MAX
						//currentDist = currentDist + Center_to_Cam_distA;
						if(Center_to_Cam_distA < currentDist && Center_to_Cam_distA != 0){ //REMOVE CASE WHERE WE CHECK SAME DATA
							currentDist = Center_to_Cam_distA;
						}						
							
					}//end inner for loop (check with all other combinations centers)

					if(currentDist > currentBiggestDist){
						currentBiggestDist = currentDist;
						biggestDistID = i;					
					}
					if(currentDist < currentSmallerDist){
						currentSmallerDist = currentDist;
						smallerDistID = i;					
					}
					if(currentBiggestDist/(combinations.size()-1) > distThreshold){
						outofThresCount++;					
					}				
				}//end for loop	

				//CHECK SECOND BIGGEST DISTANCE TO BIGGEST, if close assume more than one errorneous
				int found =0;
				for (int i = 0; i < combinations.size(); i++) {
					//find center average
					Vec3d center1;
					Vec3d center1ROT;
					for (int i1 = 0; i1 < combinations[i].size(); i1++) {
						add_to_31_matrix(center1,centerPointsTMP[combinations[i][i1]]);						
						
						Vec3d toEuler = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[i][i1]])*(180.0f/PI);	
						//cv::Mat R;
						//cv::Rodrigues(centerRotationsTMP[combinations[i][i1]],R);
						add_to_31_matrix(center1ROT, matrix31_abs(toEuler));						
						//cout << "toEuler Angles = " << toEuler << endl; 
					}
					scale31_matrix(center1, combinations[i].size());					
					scale31_matrix(center1ROT, combinations[i].size());					
					//cout << "toEuler Angles CENTER = " << center1ROT << endl; 
				
					float currentDist = 0;	//find distance to all others
					for (int j = 0; j < combinations.size(); j++) { //for (int j = i+1; j < combinations.size(); j++) {
						//find center average
						Vec3d center2;
						Vec3d center2ROT;
						for (int j1 = 0; j1 < combinations[j].size(); j1++) {
							add_to_31_matrix(center2,centerPointsTMP[combinations[j][j1]]);
							Vec3d toEuler1 = rotationMatrixToEulerAngles(centerRotationsTMP[combinations[j][j1]])*(180.0f/PI);
							add_to_31_matrix(center2ROT, matrix31_abs(toEuler1));								
						}
						scale31_matrix(center2, combinations[j].size());						
						scale31_matrix(center2ROT, combinations[j].size());						

						float Center_to_Cam_distA = matrix31_distance(center1ROT,center2ROT);						
						
						//MIN MAX
						//currentDist = currentDist + Center_to_Cam_distA;
						if(Center_to_Cam_distA < currentDist){
							currentDist = Center_to_Cam_distA;
						}						
							
					}//end inner for loop (check with all other combinations centers)
					
					//CHANGE HERE
					//if distance above threshold look to see how far biggest distances are from the biggest found above
					if(abs(currentDist - currentBiggestDist) < 45.04f){ 
						found++;					
					}
				}	

				//if max distance average above a threshold, assume we have one error, 
				//otherwise assume no erroneous marker measurement or move to find 2-3-4 etc erroneous combos
				//if(currentBiggestDist/ (combinations.size()) > 0.17f){ //  if(currentBiggestDist/ (combinations.size()-1) > 0.05f){
				//if(outofThresCount > 0 && currentBiggestDist > distThreshold){
				//if(outofThresCount > 0 && currentBiggestDist/(combinations.size()-1) > distThreshold){
				//if(currentBiggestDist/(combinations.size()-1) > distThreshold){		
					//find erroneous marker
					int errorIter = -1;
					for (int i = 0; i < centerPointsIDsTMP.size(); i++) {//search the pool of all marker IDs found for the erroneous
						bool found1 = false;
						for (int j = 0; j < combinations[biggestDistID].size(); j++) {
							if(combinations[biggestDistID][j] == centerPointsIDsTMP[i]){
								found1 = true;
							}
						}
						if(!found1){ //if centerPointsID not in the max distance group, it is the erroneous one
							errorID = centerPointsIDsTMP[i];
							errorIter = i;
							break;
						}					
					}

					//Centers comparisson with and without the outlier
					//MIN MAX - if center with outlier VS center without distance lower than threshold, stop

					vector<int> centerPointsIDsORIG = centerPointsIDsTMP;
					vector<cv::Vec<double, 3>>  centerPointsORIG = centerPointsTMP;
					//std::vector<cv::Mat> centerRotationsORIG = centerRotationsTMP;
					centerPointsIDsORIG.erase(centerPointsIDsORIG.begin() + errorIter);
					centerPointsORIG.erase(centerPointsORIG.begin() + errorIter);

					cv::Vec<double, 3> centerMean;					
					for (int i = 0; i < centerPointsIDsORIG.size(); i++) {
						add_to_31_matrix(centerMean, centerPointsORIG[i]);					
					}
					scale31_matrix(centerMean,centerPointsORIG.size());
					
					if(centerPointsIDsORIG.size() == 1){
						matrix32_assign_from_to(centerPointsORIG[0],centerMean);
					}

					cv::Vec<double, 3> centerMean1;					
					for (int i = 0; i < centerPointsIDsTMP.size(); i++) {
						add_to_31_matrix(centerMean1, centerPointsTMP[i]);							
					}
					scale31_matrix(centerMean1, centerPointsIDsTMP.size());
					
					if(centerPointsIDsTMP.size() == 1){
						matrix32_assign_from_to(centerPointsTMP[0],centerMean1);			
					}					
					
					float centersWithAndWithoutDistance = matrix31_distance(centerMean,centerMean1);
					
					//if(nCombinations < 3){  //if(centersWithAndWithoutDistance > distThreshold){
					//if(centersWithAndWithoutDistance > distThreshold){
					//if(nCombinations > 3 && centersWithAndWithoutDistance > distThreshold){
					//if(nCombinations > 3){ //if(nCombinations > 2 && centersWithAndWithoutDistance > distThreshold){

					// if min to max distance vs threshold small, do no action
					if (abs(currentBiggestDist - currentSmallerDist) < 2.04f){ //CHANGE HERE
						// NO ACTION
						//if(preview == 1){
							//cout << "currentBiggestDist = " << currentBiggestDist 
							//<< " currentSmallerDist = " << currentSmallerDist << endl; 
						//}
					}else if (found == 1){
						//cout << "LAST ERROR ID = " << errorID << " MAX DISTANCE = " 
						//<< currentBiggestDist << " DISTANT COUNT = " << outofThresCount << endl;
 
						outlierComboIDs.push_back(errorID);

						//reconstruct the matrices without the found erroneous marker
						centerPointsIDsTMP.erase(centerPointsIDsTMP.begin() + errorIter);
						centerPointsTMP.erase(centerPointsTMP.begin() + errorIter);
						centerRotationsTMP.erase(centerRotationsTMP.begin() + errorIter);
					}else if(1==1){
						biggestDistID = 0;
						float currentMaxCenterDist = 0;
						//CHOOSE HIGEST DIST POINT - ROTATIONS
						for (int i = 0; i < centerRotationsTMP.size(); ++i) 
						{ 
							//CHOOSE HIGEST DIST POINT - ROTATIONS
							float toAllDist = 0;
							for (int j = 0; j < centerRotationsTMP.size(); ++j) 
							{
							   float Center_to_Prev_Center_dist = matrix33_distance(centerRotationsTMP[j], centerRotationsTMP[i]);
							   toAllDist = toAllDist + Center_to_Prev_Center_dist;								
							}

							//distROT = centerRotationsTMP[i].at<double>(0,0);
							if(toAllDist > currentMaxCenterDist){
							   currentMaxCenterDist = toAllDist;
							   biggestDistID = i;					
							}
						}

						//WE ASSUME AL LEAST TWO ERRONEUS HERE, choose how to eliminate one so we iterate the rest
						//cout << "ASSUMED ERROR ID = " << centerPointsIDsTMP[biggestDistID] << endl; 
						outlierComboIDs.push_back(centerPointsIDsTMP[biggestDistID]);

						//reconstruct the matrices without the found erroneous marker
						centerPointsIDsTMP.erase(centerPointsIDsTMP.begin() + biggestDistID);
						centerPointsTMP.erase(centerPointsTMP.begin() + biggestDistID);
						centerRotationsTMP.erase(centerRotationsTMP.begin() + biggestDistID);

						//cout << "ASSUMED ERROR ID A = " << centerPointsIDsTMP[biggestDistID] << endl; 
						discoverOutliersMinMax( centerPointsTMP,centerPointsIDsTMP,centerRotationsTMP );
						//cout << "ASSUMED ERROR ID B = " << comboDepthLevel << " _____" << centerPointsIDsTMP[biggestDistID] << endl; 
					}
					// if min to max distance vs threshold large and if only one distant, assing one erroneuous and stop
					// if min to max distance vs threshold large and if more than one distant, search to remove one marker and loop
					
				//}
}//END FUNCTION 2 MIN MAX


//// ARUCO PARAMETERS
float markerLength ;
int preview;
Mat camMatrix, distCoeffs;
String video;
VideoCapture inputVideo;

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());     
    return  norm(I, shouldBeIdentity) < 1e-6;     
}

//////////////// ARUCO MARKER ID /////////////
vector<cv::Vec<double, 3>> centerPoints;
vector<int> centerPointsIDs;
std::vector<cv::Mat> centerRotations;

Vec3f toEuler; //toEuler = rotationMatrixToEulerAngles(rotResultC)*(180.0f/PI);
int counted = 0;
Vec3d rvecsBaseC, tvecsBaseC;
Mat rotResultC = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
//separate inliers and outliers
vector< int > inlierIDs;
vector< int > outlierIDs;
std::string individual_identified_tranforms_OUTPUT;
Mat transResultC = (Mat_<double>(1,3) << 0,0,0);
float distSideToSolidCenter;
float actualRectSideSize;
//summaries
int identifiedTriangleCount=0;
int identifiedRectangleCount=0;

vector< int > ids;
float Center_to_Cam_dist = -1;

vector< Point2f > imagePoints;	
void resetVTempVariables3(){
	cout << "centerPoints " << centerPoints.size() << "centerPointsIDs " << centerPointsIDs.size() << "centerRotations " << centerRotations.size() << "counted " << counted << endl;		

	centerPoints.clear();
	centerPointsIDs.clear();
	centerRotations.clear();
	counted = 0;

	inlierIDs.clear();
	outlierIDs.clear();
	identifiedTriangleCount=0;
	identifiedRectangleCount=0;	
	Center_to_Cam_dist = -1;
	imagePoints.clear();

	outlierComboIDs.clear();
	comboIteration=0;
	foundIDs.clear();
	combination.clear();
	combinations.clear();
	errorID = -1;
	comboDepthLevel=0;

	rotResultC = (Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
	transResultC = (Mat_<double>(1,3) << 0,0,0);
}

void arucoProcessmarkerID(Mat& imageCopy, vector< int > ids, vector<Vec3d> rvecs, vector<Vec3d> tvecs, float tileSize, vector< int > idsMAP) {

	actualRectSideSize = 0.074f;
	distSideToSolidCenter = ((sqrt(2.0f)+1.0f)/2.0f) * actualRectSideSize;

	//ANALYSE FOUND MARKERS
	if (ids.size() > 0) {
		for (int i = 0; i < ids.size(); i++) {
			//cam pose
			cv::Mat R;
			cv::Rodrigues(rvecs[i],R);
			cv::Mat camPose = -R.t() * (cv::Mat)tvecs[i];
			
			//find axis in relation to camera space for Z axis (0,0,0 to 1,1,1)
			cv::Vec<double, 3> axisPoints0;
			axisPoints0(0)=0;
			axisPoints0(1)=0;
			axisPoints0(2)=0;
			cv::Vec<double, 3> axisPoints1;
			axisPoints1(0)=0;
			axisPoints1(1)=0;
			axisPoints1(2)=1;
			//rotate points to camera space, to find angle difference based on common axis
			cv::Mat RotatedToCameraSpaceaxisPoints0 = R * (cv::Mat)axisPoints0;
			cv::Mat RotatedToCameraSpaceaxisPoints1 = R * (cv::Mat)axisPoints1;
			
			int centralID = idsMAP[0];
			int leftID1 = idsMAP[1];
			int leftID2 = idsMAP[2];
			int leftID3 = idsMAP[3];
			int leftID4 = idsMAP[4];
			int leftID5 = idsMAP[5];
			int leftID6 = idsMAP[6];
			int leftID7 = idsMAP[7];			
			int centralUpperID = idsMAP[8]; 
			int leftUpperTriangleID1 = idsMAP[9];
			int leftUpperID1 = idsMAP[10];
			int leftUpperTriangleID2 = idsMAP[11];
			int leftUpperID2 = idsMAP[12];
			int leftUpperTriangleID3 = idsMAP[13];
			int leftUpperID3 = idsMAP[14];
			int leftUpperTriangleID4 = idsMAP[15];
			int topID = idsMAP[16];	
			
			//plot axis of solid center
			Vec3d rvecsBase, tvecsBase;				

			if((ids[i] != leftUpperTriangleID1) && ids[i] != leftUpperTriangleID2 && ids[i] != leftUpperTriangleID3  && (ids[i] != leftUpperTriangleID4)){

				//rotate final axis based on rotation vs centralID marker axis				
				Mat rotMatrixFinal = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1); 
				float th = 0;
				if(ids[i] == leftID1){ 
					//around Y, 45 degrees					
					th = -1.0f*(PI/4.0f);									
				}
				if(ids[i] == leftID2 || ids[i] == leftUpperID1){ 
					//around X, 90 degrees
					th = -2.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID3){ 
					//around Y, 45 degrees
					th = -3.0f*(PI/4.0f);										
				}
				if(ids[i] == leftID4 || ids[i] == leftUpperID2){ 
					//around Y, 45 degrees
					th = -4.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID5){ 
					//around Y, 45 degrees					
					th = -5.0f*(PI/4.0f);									 
				}
				if(ids[i] == leftID6 || ids[i] == leftUpperID3){ 
					//around Y, 45 degrees
					th = -6.0f*(PI/4.0f);					
				}
				if(ids[i] == leftID7){ 
					//around Y, 45 degrees					
					th = -7.0f*(PI/4.0f);										
				}
				
				rotMatrixFinal = (Mat_<double>(4,4) << cos(th),0,sin(th),0, 0,1,0,0, -sin(th),0,cos(th),0, 0,0,0,1); 

				//ROTATE UPPER PARTS around X axis (rotated in Y axis above already)
				if(ids[i] == leftUpperID1 || ids[i] == leftUpperID2 || ids[i] == leftUpperID3){ 
						th = 1.0f*(PI/4.0f);
						rotMatrixFinal = 
						(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1)*rotMatrixFinal;
				}
				if(ids[i] == centralUpperID){ 

					th = 1.0f*(PI/4.0f);
					
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1);
				}						
				if(ids[i] == topID){ 
					th = 2.0f*(PI/4.0f);
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1);
				}				
								
				//cast translation vector to matrix				
				Mat translateMatrix = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,-distSideToSolidCenter, 0,0,0,1); 

				//rot
				Mat rotMatrix = (Mat_<double>(4,4) << 
						R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),0, 
						R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),0, 
						R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),0, 
						0,0,0,1);

				//homogenous transf						
				Mat transHomog = (Mat_<double>(4,1) << tvecs[i](0),tvecs[i](1),tvecs[i](2),1);	
				Mat translateCamMarker = (Mat_<double>(4,4) << 
				1,0,0,tvecs[i](0), 
				0,1,0,tvecs[i](1), 
				0,0,1,tvecs[i](2), 
				0,0,0,1);					
				
				//create rotation matrix, do translation, rotation, translation, then split matrix back to rvec,tvec				
				Mat centerPose = translateCamMarker * rotMatrix * translateMatrix * rotMatrixFinal; // R * (cv::Mat)tvecs[i];
				
				cout << tvecs[i] << endl;				
				
				//Take whole
				Mat rotResult = (Mat_<double>(3,3) << 
						centerPose.at<double>(0,0),centerPose.at<double>(0,1),centerPose.at<double>(0,2), 
						centerPose.at<double>(1,0),centerPose.at<double>(1,1),centerPose.at<double>(1,2), 
						centerPose.at<double>(2,0),centerPose.at<double>(2,1),centerPose.at<double>(2,2));
				Mat transResult = (Mat_<double>(1,3) << centerPose.at<double>(0,3),centerPose.at<double>(1,3),centerPose.at<double>(2,3));

				cout << transResult << endl;
				
				//get final rotation and turn it to vector				
				cv::Rodrigues(rotResult,rvecsBase);
				tvecsBase = transResult;				

				//consensus
				cv::Vec<double, 3> transResultVec;
				transResultVec(0)=centerPose.at<double>(0,3);
				transResultVec(1)=centerPose.at<double>(1,3);
				transResultVec(2)=centerPose.at<double>(2,3);
				centerPoints.push_back(transResultVec);	
				centerRotations.push_back(rotResult);	
				centerPointsIDs.push_back(ids[i]);				

			}// END IDs check to not be triangles (cover only rectangles)
			else { //if NOT Cube handle triangles

				//triangle summaries
				identifiedTriangleCount++;

				//handle triangles				
				//Mesokathetos trigonou SideX
				float SideX = actualRectSideSize*(sqrt(3.0f)/2.0f);
		
				//Inner marker side half size SideC (SideB/2)
				float SideC = (0.5f*sin(PI/3.0f)* actualRectSideSize)/(1.0f+sin(PI/3.0f));

				//Perperndicular from Center to mesokathetos end point to triangle upper corner point
				float SideDA = 0.2064f * actualRectSideSize * sqrt(5.0f+2.0f*sqrt(2.0f));
				
				//Perperndicular line length from Center to mesokathetos 
				float SideTR = 0.45542f * actualRectSideSize * sqrt(5.0f+2.0f*sqrt(2.0f));

				//Distance from marker center to perperndicular from Center to mesokathetos end point
				float SideW = SideX -SideDA -SideC;

				Mat rotMatrixFinal = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1); 
				float th = 0;
				if(ids[i] == leftUpperTriangleID1){ 
					//around Y, 45 degrees
					th = -1.0f*(PI/4.0f);					
				}				
				if(ids[i] == leftUpperTriangleID2){ 
					//around Y, 45 degrees
					th = -3.0f*(PI/4.0f);					
				}				
				if(ids[i] == leftUpperTriangleID3){ 
					//around Y, 45 degrees
					th = -5.0f*(PI/4.0f);				
				}				
				if(ids[i] == leftUpperTriangleID4){ 
					//around Y, 45 degrees
					th = -7.0f*(PI/4.0f);					
				}
				rotMatrixFinal = (Mat_<double>(4,4) << cos(th),0,sin(th),0, 0,1,0,0, -sin(th),0,cos(th),0, 0,0,0,1); 

				//ROTATE UPPER PARTS around X axis (rotated in Y axis above already)
				if(ids[i] == leftUpperTriangleID1 || ids[i] == leftUpperTriangleID2 || ids[i] == leftUpperTriangleID3 
				|| ids[i] == leftUpperTriangleID4)
				{ 					
					th = (35.26f/180.0f)*PI; //(54.73f/180.0f)*PI; //1.0f*(PI/4.0f); // (45/180)*PI // (35.26f/180.0f)*PI;
					rotMatrixFinal = 
					(Mat_<double>(4,4) << 1,0,0,0, 0, cos(th),-sin(th),0, 0, sin(th),cos(th),0, 0,0,0,1)*rotMatrixFinal;
				}				

				//FIND large to small fiducial marker ration
				float ratio = 1.0;  //6.9f / 3.4f; //3.3f; // CHANGE HERE (last was 3.4f)							
				
				//CHECK if dist is same
				Mat translateMatrix = (Mat_<double>(4,4) << 1,0,0,0, 0,1,0,SideW, 0,0,1,-SideTR, 0,0,0,1);

				//rot
				Mat rotMatrix = (Mat_<double>(4,4) << 
						R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),0, 
						R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),0, 
						R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),0, 
						0,0,0,1);

				//homogenous transf				
				Mat translateCamMarker = (Mat_<double>(4,4) << 
						1,0,0,tvecs[i](0)/ratio, 0,1,0,tvecs[i](1)/ratio, 0,0,1,tvecs[i](2)/ratio, 0,0,0,1);					
				
				//create rotation matrix, do translation, rotation, translation, then split matrix back to rvec,tvec				
				Mat centerPose = translateCamMarker * rotMatrix * translateMatrix * rotMatrixFinal; // R * (cv::Mat)tvecs[i];
				
				//Take whole
				Mat rotResult = (Mat_<double>(3,3) << 
						centerPose.at<double>(0,0),centerPose.at<double>(0,1),centerPose.at<double>(0,2), 
						centerPose.at<double>(1,0),centerPose.at<double>(1,1),centerPose.at<double>(1,2), 
						centerPose.at<double>(2,0),centerPose.at<double>(2,1),centerPose.at<double>(2,2));
				Mat transResult = (Mat_<double>(1,3) << centerPose.at<double>(0,3),centerPose.at<double>(1,3),centerPose.at<double>(2,3));

				
				//get final rotation and turn it to vector				
				cv::Rodrigues(rotResult,rvecsBase);
				tvecsBase = transResult;
				
				//consensus
				cv::Vec<double, 3> transResultVec;
				transResultVec(0)=centerPose.at<double>(0,3);
				transResultVec(1)=centerPose.at<double>(1,3);
				transResultVec(2)=centerPose.at<double>(2,3);
				centerPoints.push_back(transResultVec);	
				centerRotations.push_back(rotResult);	
				centerPointsIDs.push_back(ids[i]);		
			}		
			
			vector< Point2f > imagePointsA;	
			vector< Point3f > axisPoints;
			axisPoints.push_back(Point3f(0, 0, 0));
			axisPoints.push_back(Point3f(markerLength * 7.5f, 0, 0));
			axisPoints.push_back(Point3f(0, markerLength * 7.5f, 0));
			axisPoints.push_back(Point3f(0, 0, markerLength * 7.5f));			
			
			Mat imagePointsVelocities = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); 
			
			projectPoints(axisPoints, rvecsBase, tvecsBase, camMatrix, distCoeffs, imagePointsA, imagePointsVelocities);
			if(1==0 && preview == 1 && imagePointsA.size() > 1){								
							// draw axis lines
							if(imagePointsA[0].x>0 && imagePointsA[0].x < camWidth && imagePointsA[0].y>0 && imagePointsA[0].y < camHeight &&
							imagePointsA[1].x>0 && imagePointsA[1].x < camWidth && imagePointsA[1].y>0 && imagePointsA[1].y < camHeight &&
							imagePointsA[2].x>0 && imagePointsA[2].x < camWidth && imagePointsA[2].y>0 && imagePointsA[2].y < camHeight &&
							imagePointsA[3].x>0 && imagePointsA[3].x < camWidth && imagePointsA[3].y>0 && imagePointsA[3].y < camHeight
							){
								line(imageCopy, imagePointsA[0], imagePointsA[1], Scalar(0, 0, 255/(i+1)), 3);
								line(imageCopy, imagePointsA[0], imagePointsA[2], Scalar(0, 255/(i+1), 0), 3);
								line(imageCopy, imagePointsA[0], imagePointsA[3], Scalar(255/(i+1), 0, 0), 3);	
							}	
							imshow("Alll axis",imageCopy);
			}			

		}//END FOR LOOP over IDs found
	}//END check if ids found > 0
}//END ARUCO MARKER IDENTIFICATION FUNCTION



////////////////// DO OUTLIERS	
void doOuliers(){
	vector< Point3f > axisPoints;
	axisPoints.push_back(Point3f(0, 0, 0));
	axisPoints.push_back(Point3f(markerLength * 7.5f, 0, 0));
	axisPoints.push_back(Point3f(0, markerLength * 7.5f, 0));
	axisPoints.push_back(Point3f(0, 0, markerLength * 7.5f));		
	imagePoints.clear();
  	if (centerPointsIDs.size() > 0) {	
			
		//CONSENSUS
		//separate inliers and outliers		
		inlierIDs.clear();
		outlierIDs.clear();
		int consensusMethod = 2;
		cv::Vec<double, 3> centerMean;								

		//METHOD 1
		consensusMethod = 1;		

		//METHOD 1 PLOT
		if(consensusMethod == 1){

			outlierComboIDs.clear(); //empty the erroneous
			comboIteration = 0; //reset iterations count					

			errorID = -1;
			comboDepthLevel = 0;

			//discoverOutliers();
			vector<cv::Vec<double, 3>> centerPointsTMP = centerPoints;
			vector<int> centerPointsIDsTMP = centerPointsIDs;
			std::vector<cv::Mat> centerRotationsTMP = centerRotations;
			
			discoverOutliersMinMax( centerPointsTMP,centerPointsIDsTMP,centerRotationsTMP ); // FIND OUTLIERS
			
			centerPoints    = centerPointsTMP;
			centerPointsIDs = centerPointsIDsTMP;
			centerRotations = centerRotationsTMP;
			zero31_matrix(centerMean);		
			
			inlierIDs.clear();
			outlierIDs.clear();

			for (int i = 0; i < centerPointsIDs.size(); i++) {				
				float Center_to_Prev_Center_dist = matrix31_distance(centerMean,centerPoints[i]);
				
				//threshold, if center is above this will not use point in consensus
				if(Center_to_Prev_Center_dist < 0.06){ //if(Center_to_Prev_Center_dist < 0.05 && pass0){  //CHANGE HERE
					transResultC.at<double>(0,0) = centerPoints[i](0) + transResultC.at<double>(0,0);
					transResultC.at<double>(0,1) = centerPoints[i](1) + transResultC.at<double>(0,1);
					transResultC.at<double>(0,2) = centerPoints[i](2) + transResultC.at<double>(0,2);
					//cout << "centerPoints[" << i << "] = " << centerPoints[i] << endl;					
					counted=counted+1;

					inlierIDs.push_back(centerPointsIDs[i]);
				}else{
					//flip point and add
					//translate along its z axis double the distance
					if(i == centerPointsIDs.size()-1 && inlierIDs.size() == 0){
						transResultC.at<double>(0,0) = centerPoints[i](0) + transResultC.at<double>(0,0);
						transResultC.at<double>(0,1) = centerPoints[i](1) + transResultC.at<double>(0,1);
						transResultC.at<double>(0,2) = centerPoints[i](2) + transResultC.at<double>(0,2);
						inlierIDs.push_back(centerPointsIDs[i]);
						counted=counted+1;				//IF last element and no inliers, choose as center
					}else{
						outlierIDs.push_back(centerPointsIDs[i]);
					}
				}
			}

			if(counted == 0){
				transResultC.at<double>(0,0) = 0;
				transResultC.at<double>(0,1) = 0;
				transResultC.at<double>(0,2) = 0;
				tvecsBaseC = transResultC;
				//return 0;
			}else{
				transResultC.at<double>(0,0) = (transResultC.at<double>(0,0)) / counted;
				transResultC.at<double>(0,1) = (transResultC.at<double>(0,1)) / counted;
				transResultC.at<double>(0,2) = (transResultC.at<double>(0,2)) / counted;
				tvecsBaseC = transResultC;//redo passing to final axis vector
			}

			//ROTATIONS			
			zero33_matrix(rotResultC);

			//cout << "AFTER ZEROING = " << rotResultC << endl;
			Mat rotResultCTMP = (Mat_<double>(3,3) << 0,0,0, 0,0,0, 0,0,0);
			for (int i = 0; i < centerPointsIDs.size(); i++) {	
				add_to_33_matrix (rotResultCTMP, centerRotations[i]);				
				//cout << "Center ID = " << centerPointsIDs[i] << " Center ID Coords = " << centerPoints[i] << endl;
			}
			scale33_matrix(rotResultCTMP, centerPointsIDs.size());
			
			//cout << "Center MEAN = " << centerMean << endl;
			//If distance from mean center above threshold, dont count in centre calculation (reset previous loop based created consensus)
			int countedRots = 0;
			for (int i = 0; i < centerPointsIDs.size(); i++) {
				float Center_to_Prev_Center_dist = matrix33_distance(rotResultCTMP, centerRotations[i]);				
				//if(Center_to_Prev_Center_dist < 1.05){ //threshold, if center is above this will not use point in consensus
					add_to_33_matrix (rotResultC, centerRotations[i]);					
					countedRots=countedRots+1;
				//}
			}
			scale33_matrix(rotResultC, countedRots);			
			cv::Rodrigues(rotResultC,rvecsBaseC);//redo passing to final rotations vector	
			Mat imagePointsVelocities = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); //

			//PROJECT SOLID CENTER AXIS			
			projectPoints(axisPoints, rvecsBaseC, tvecsBaseC, camMatrix, distCoeffs, imagePoints, imagePointsVelocities);			

		}//END METHOD 1 PLOTS 		
		//KEEP PREVIOUS CENTERS and their PROJECTIONS		
	}
}// END DO OUTLIERS



///ARUCO3
void initAruco3(){
	Mat init = Mat::eye(4,4, CV_32F);
	for(int i = 0 ; i < 3 ; i++){
		Final_Results.push_back(init); // initialized as identity matrices
	}
	Mdetector.setDictionary("ARUCO_MIP_36h12", 0.f);

	markerLength = 0.01 ;
	double focal_len = 476.7030836014194 ;	double cxy = 400.5 ;
	//Mat camMatrix  = (Mat1d(3,3) <<  focal_len , 0.0 , cxy , 0.0 , focal_len , cxy , 0.0 , 0.0 , 1.0) ;
	//Mat distCoeffs = (Mat1d(1,4) << 0.0 , 0.0 , 0.0 , 0.0) ; 
	Mat camMatrix1  = camMatrix;
	Mat distCoeffs1 = distCoeffs;
	Size camSize = Size(1920, 1080);
	CamParam.setParams(camMatrix1, distCoeffs1, camSize);
}

//////////////// END ARUCO MARKER ID /////////
bool isTriangle(int id){
	
	bool result;
	int index = (id-94)*(id-97)*(id-100)*(id-103)*(id-149)*(id-152)*(id-155)*(id-158)*(id-139)*(id-142)*(id-145)*(id-148);
	if(index==0){
		result = true;	
	}else{
		result = false;
	}
	return result;
}


bool arucoProcess(Mat srcImg) {
	
   	//OCAM - ARUCO
	Mat imageCopy;	
	
	//PLOT ALL AXIS
	Mat allCentersImage;
	srcImg.copyTo(allCentersImage);	

	//MARKER SIZE DEFINITION
	float tileSize = 0.070f; //final solid size

		//////////////ARUCO3
		Mat input_image;
		input_image = srcImg;
		input_image.copyTo(imageCopy);

		//MAPPING
		vector<vector<int>> idsMAPALL;
		//vector<int> idsMAPALL_SOLID1;
		vector<int> idsMAPALL_SOLID1{ 33, 24, 36, 3, 6, 9, 0, 30, 27, 48, 12, 39, 18, 42, 21, 45, 15 }; //put solid 1 markers IDs in the specific order (x17)		
		idsMAPALL.push_back(idsMAPALL_SOLID1);
		vector<int> idsMAPALL_SOLID2{ 76, 79, 91, 55, 85, 67, 64, 73, 82, 97, 88, 100, 58, 94, 61, 103, 70 }; //put solid 1 markers IDs in the specific order (x17)		
		idsMAPALL.push_back(idsMAPALL_SOLID2);
		vector<int> idsMAPALL_SOLID3{ 140, 116, 119, 134, 146, 143, 131, 110, 113, 152, 122, 158, 137, 155, 128, 149, 125 }; //put solid 1 markers IDs in the specific order (x17)		
		idsMAPALL.push_back(idsMAPALL_SOLID3);
			
		vector<Marker> Markers = Mdetector.detect(input_image); // find all markers in frame
		if(Markers.size()>0){ // do nothing if no markers were found
			
			Markers_List marker_list = separate_markers(Markers); // separate them
			vector< vector<Marker> > rhombi_markers = marker_list.markers_list ; // vector of marker vectors, each element is a vector of markers of one rhombi
			vector<int> flags = marker_list.flags ; // vector of rhombi identifiers. flags[i] is the rhombi identity of rhombi_markers[i].
		
			CamParam.resize(input_image.size());
			
			for (int i = 0; i < flags.size() ; i++){ // for every different rhombi seen (shouldn't be more than two)			
				vector<Marker> rhombi_ = rhombi_markers[i] ; // get vector of detected markers of this rhombi 
				
				int count = 0;
				vector<int> tmpIDs;	
				vector<Vec3d> tmpRvecss;
				vector<Vec3d> tmpTvecss;
				
				for (auto& marker : rhombi_ ){ // for every one of them
					float realMarkerLength;
					if(isTriangle(marker.id)){
						realMarkerLength = 0.033;
					}else{
						realMarkerLength = 0.07;
					}
			    	Mtracker[marker.id].estimatePose(marker, CamParam, realMarkerLength, 12 );  // call its tracker and estimate the pose
			    	if (CamParam.isValid() && markerLength != -1){
		        		if (marker.isPoseValid()){
		            		CvDrawingUtils::draw3dAxis(input_image, marker, CamParam); // this is just for drawing
		        		}
		        	}
	 
					if(marker.isPoseValid()){ // if you successfully posed the marker
						count++;
						tmpIDs.push_back(marker.id);
						tmpRvecss.push_back(marker.Rvec);
						tmpTvecss.push_back(marker.Tvec);
					}
				}
				if(count>0){
					resetVTempVariables3();
					arucoProcessmarkerID(imageCopy, tmpIDs, tmpRvecss, tmpTvecss, tileSize, idsMAPALL[flags[i]]); //find solid 0 points
					//process solid 0 points and plot it
					doOuliers();	
					//plot base axis
					if(preview == 1 && imagePoints.size() > 1){			
							// draw axis lines
							if(imagePoints[0].x>0 && imagePoints[0].x < camWidth && imagePoints[0].y>0 && imagePoints[0].y < camHeight &&
							imagePoints[1].x>0 && imagePoints[1].x < camWidth && imagePoints[1].y>0 && imagePoints[1].y < camHeight &&
							imagePoints[2].x>0 && imagePoints[2].x < camWidth && imagePoints[2].y>0 && imagePoints[2].y < camHeight &&
							imagePoints[3].x>0 && imagePoints[3].x < camWidth && imagePoints[3].y>0 && imagePoints[3].y < camHeight
							){
								line(allCentersImage, imagePoints[0], imagePoints[1], Scalar(0, 0, 255/(i+1)), 3);
								line(allCentersImage, imagePoints[0], imagePoints[2], Scalar(0, 255/(i+1), 0), 3);
								line(allCentersImage, imagePoints[0], imagePoints[3], Scalar(255/(i+1), 0, 0), 3);	
							}	
							//imshow("out"+to_string(outWindowID), imageCopy);		
					}				
				}				
			}	
		
			for ( int i = 0 ; i < Markers.size() ; i++ ) {
				Markers[i].draw(input_image, Scalar(0, 0, 255), 2);			
			}        

	  		//imshow("frame",input_image);
			
		}//END if(Markers.size()>0){
		if(preview == 1){
					imshow("outALL", allCentersImage);
					//Mat imageUndistorted;
					//undistort(image, imageUndistorted,camMatrix, distCoeffs);
					//imshow("outUndistorted", imageUndistorted);
		} 	

	return 1;	
}

//// END ARUCO


//CHECK IMAGE EQUALITY
bool equal(const Mat& a, const Mat& b)
{
    if ( (a.rows != b.rows) || (a.cols != b.cols) )
        return false;
    Scalar s = sum( a - b );
    return (s[0]==0) && (s[1]==0) && (s[2]==0);
}


//////// ROS
CommandLineParser parser();

int frameIDS = 0;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) //(const sensor_msgs::ImageConstPtr& msg, int image360part)
{		
  try
  {		
	cv::Mat srcImg, dstImg;
	srcImg = cv_bridge::toCvShare(msg, image_encoding)->image;
        dstImg = srcImg;
	cv::imshow("view", srcImg);
	cv::waitKey(1);	
	//usleep(1);
	
	frameIDS++;
	if(!srcImg.empty()){		
		arucoProcess(srcImg);		
	}//! empty check

  }
  catch (cv_bridge::Exception& e)
  {
    	//ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	ROS_ERROR("Could not convert from to 'bgr8'.");
  }
}


//sample counter
int POZYXsamplesCounted = 0;

//MAIN
int main(int argc, char **argv)
{  
	ros::init(argc, argv, "aruco_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    	//image_transport::Subscriber sub = it.subscribe("/flir_adk/image_raw", 1, imageCallback);
	image_transport::Subscriber sub = it.subscribe("/camera_array/cam0/image_raw", 1, imageCallback);

	//ARUCO START
	//CommandLineParser parser(argc, argv, keys);	
	//markerLength = parser.get<float>("l");	
	//preview = parser.get<int>("p");
	preview=0;
        markerLength=0.07;
	if(preview == 1){
		cv::namedWindow("view");
		cv::startWindowThread();
	}

	//ARUCO PARAMS
	//if(parser.has("c")) {
	//	bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
	//	if(!readOk) {
	//		cerr << "Invalid camera file" << endl;
	//		return 0;
	//	}
	//}
 
	// String video;
	//if(parser.has("v")) {
	//	video = parser.get<String>("v");
	//}

	//if(!parser.check()) {
	//	parser.printErrors();
	//	return 0;
	//}

	initAruco3();

	//NEW1
	//VideoCapture cap;
	
	//cap.open(0,CAP_V4L);
	// Exit if video is not opened
	//if(!cap.isOpened())
	//{
	//	cout << "Could not read video file" << endl; 
	//	return 1; 
	//} 
	
	//float resMultiplier = 3;
	//cap.set(3,640*resMultiplier);
	//cap.set(4,360*resMultiplier);
	//int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
	//int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	//assert(cap.isOpened());		
	//Mat temp1; cap >> temp1;
	ros::Rate therate= 10; //Hz
	while(ros::ok() && true){
		//cap >> temp1;
		//imageCallback(temp1);
	    	ros::spinOnce();
		therate.sleep();
	}
	//END NEW1	
}
