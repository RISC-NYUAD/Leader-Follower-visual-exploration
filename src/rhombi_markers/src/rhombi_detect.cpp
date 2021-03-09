#include "rhombi_detect.h"

using namespace std;
using namespace aruco;	//ARUCO3
using namespace cv;

bool experiment_play=false;

MarkerDetector Mdetector;
aruco::CameraParameters CamParam;

int frameIDS;

marker_geom MarkerRhombiGeometries_;
std::vector<rhombi_detect> rtvecs;

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

cv::Matx41d aa2quaternion(const Matx31d& aa)
{
	double angle = norm(aa);
	Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
	double angle_2 = angle / 2;
	//qx, qy, qz, qw
	Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
	return q;
}

bool inRange(int low,  int high,  int x)
{
	return (low < x && x < high);
}

double compute_distance(cv::Mat1d tvec_prev, cv::Mat1d tvec_new ){
	return sqrt(pow(tvec_prev(0,0)-tvec_new(0,0),2)+pow(tvec_prev(0,1)-tvec_new(0,1),2)+pow(tvec_prev(0,2)-tvec_new(0,2),2));
}

void solvePNP_NE(cv::Mat srcImg){
	// Copy image to visualize Axis and/or save
	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	if(contrast != 1.0 || brightness != 0.0){
		srcImg.convertTo(srcImg,-1,contrast,brightness);
	}
	std::vector<Marker> Markers = Mdetector.detect(srcImg); // find all markers in frame
	if(Markers.size()>0){
		//Draw markers on image
		cv::Mat imageAxes;// Do not delete dummy Mat for image saving to work
		if(PREVIEW || SAVE_IMAGES){
			cv::Mat imageAxes_(srcImg.size(), CV_8UC3);
			cv::cvtColor(srcImg, imageAxes_, CV_GRAY2RGB);
			imageAxes = imageAxes_;
			for(auto m : Markers){
				m.draw(imageAxes, Scalar(0, 0, 255), 2);
			}
		}
		
		//Prepare publishing message -- Time in seconds since EPOCH - post processing required to get elapsed
		pose_publishing.header.seq = frameIDS;
		pose_publishing.header.stamp = ros::Time::now();
		
		//For every different rhombi config check if marker exists and append
		std::vector<std::vector<aruco::Marker>> RhombiMarkerDetections(MarkerRhombiGeometries_.RhombiMarkerConfigs.size());
		for(uint8_t it1 = 0; it1 < MarkerRhombiGeometries_.RhombiMarkerConfigs.size(); it1++){
			std::vector<int> CurrentConfig = MarkerRhombiGeometries_.RhombiMarkerConfigs[it1];
			vector<cv::Point3d>  objectPoints;
			vector<cv::Point2d>  imagePoints;
			uint8_t num_detections = 0;
			for(auto k: Markers){
				std::vector<int>::iterator it = std::find(CurrentConfig.begin(), CurrentConfig.end(), k.id);
				if(it != CurrentConfig.end()){
					if(DEBUG) {std::cout << "Found " << " Marker ID: " << k.id << " from Rhombi # " << (int) it1 << std::endl;}
					int index = k.id-CurrentConfig.size()*(k.id/CurrentConfig.size());
					for(uint8_t c=0;c<4;c++){
						imagePoints.push_back(k[c]);
						objectPoints.push_back(Point3d(MarkerRhombiGeometries_.RhombiCornerGeometries[index][c*3+0],MarkerRhombiGeometries_.RhombiCornerGeometries[index][c*3+1],MarkerRhombiGeometries_.RhombiCornerGeometries[index][c*3+2]));
					}
					num_detections++;
				}
				else{
					if(DEBUG){
						ROS_WARN("Marker ID %d not matched with any Rhombis.Current config:",k.id);
						for(uint8_t i=0; i < CurrentConfig.size(); i++)	std::cout << CurrentConfig[i] << std::endl;
					}
				}
			}
			//If more than one markers found for this Rhombi run solvePnP
			if(num_detections>=min_detections){
				bool solved;
				if(DEBUG){	std::cout << imagePoints << " " << objectPoints<<std::endl;}
				//SolvePnP
				if(USE_PREV_RVEC_TVEC && !rtvecs[it1].rvec.empty()){
					if(DO_FCU_REFINEMENT){
						//TO DO - REFINE THE RVEC TVEC based ON FCU reported motion
					}
					solved = cv::solvePnP(objectPoints, imagePoints, CamParam.CameraMatrix, CamParam.Distorsion, rtvecs[it1].rvec, rtvecs[it1].tvec, true, 0);
				}
				else{
					solved = cv::solvePnP(objectPoints, imagePoints, CamParam.CameraMatrix, CamParam.Distorsion, rtvecs[it1].rvec, rtvecs[it1].tvec, false, 3);
				}
				//Check if we have detection. If yes proceed with publish
				if(solved){
					//Create geometry_msgs for publishing
					geometry_msgs::Pose _Pose;
					cv::Mat R2(3, 3, CV_64FC1);
					cv::Rodrigues(rtvecs[it1].rvec, R2);
					Eigen::Matrix3d eigMat2;
					cv::cv2eigen(R2, eigMat2);
					Eigen::Quaterniond quat2(eigMat2);
					tf::quaternionEigenToMsg(quat2 , _Pose.orientation);
					
					_Pose.position.x = rtvecs[it1].tvec(0,0);
					_Pose.position.y = rtvecs[it1].tvec(0,1);
					_Pose.position.z = rtvecs[it1].tvec(0,2);
					pose_publishing.poses.push_back(_Pose);
					
					//The usual checks
					if(DEBUG){
						cv::Mat to_matrix;
						cv::Vec3f to_euler_xyz;
						cv::Rodrigues(rtvecs[it1].rvec, to_matrix);
						to_euler_xyz=rotationMatrixToEulerAngles(to_matrix);
						std::cout << "Rhombi r_vec: " << to_euler_xyz[0]*180/M_PI << " " << to_euler_xyz[1]*180/M_PI << " " << to_euler_xyz[2]*180/M_PI << " " << std::endl;
						std::cout << "Rhombi t_vec: " << rtvecs[it1].tvec << std::endl << std::endl;
					}
					//Prepare image for preview/saving
					if(PREVIEW || SAVE_IMAGES){
						vector< Point2f > imagePointsA;	
						vector< Point3f > axisPoints;
						axisPoints.push_back(Point3f(0, 0, 0));
						axisPoints.push_back(Point3f(0.01 * 7.5f, 0, 0));
						axisPoints.push_back(Point3f(0, 0.01 * 7.5f, 0));
						axisPoints.push_back(Point3f(0, 0, 0.01 * 7.5f));			
						Mat imagePointsVelocities = (Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0); 
						projectPoints(axisPoints, rtvecs[it1].rvec, rtvecs[it1].tvec, CamParam.CameraMatrix, CamParam.Distorsion, imagePointsA, imagePointsVelocities);
						// draw axis lines
						int C_w=CamParam.CamSize.width;
						int C_h=CamParam.CamSize.height;
						if(inRange(0,C_w,imagePointsA[0].x) && inRange(0,C_h,imagePointsA[0].y) &&
							inRange(0,C_w,imagePointsA[1].x) && inRange(0,C_h,imagePointsA[1].y) &&
							inRange(0,C_w,imagePointsA[2].x) && inRange(0,C_h,imagePointsA[2].y) &&
							inRange(0,C_w,imagePointsA[3].x) && inRange(0,C_h,imagePointsA[3].y)){
							line(imageAxes, imagePointsA[0], imagePointsA[1], Scalar(0, 0, 255), 2);
						line(imageAxes, imagePointsA[0], imagePointsA[2], Scalar(0, 255, 0), 2);
						line(imageAxes, imagePointsA[0], imagePointsA[3], Scalar(255, 0, 0), 2);
							}
					}
				}
				else{
					if(DEBUG)	ROS_WARN("No solution found");
				}
			}
		}
		if(PREVIEW)	{imshow("RhombiAxes",imageAxes);	cv::waitKey(1);}
		if(SAVE_IMAGES)	cv::imwrite(image_savepath + std::to_string(frameIDS) + ".jpg",imageAxes);
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
			if(DEBUG) ROS_INFO("Got new image!");
			frameIDS = msg->header.seq;
			solvePNP_NE(srcImg);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_WARN("Could not convert from %s",image_encoding.c_str());
	}
}


void experimentHandleFeedback(std_msgs::Bool thehandle ){
	if(thehandle.data && !experiment_play){
		experiment_play = true;
	}
	else if(!thehandle.data && experiment_play){
		experiment_play = false;
	}
	else experiment_play =true;
}

//MAIN
int main(int argc, char **argv)
{  
	//Init ROS
	ros::init(argc, argv, "rhombi_detect");
	ros::NodeHandle nh("~");
	//Get PARAMS from server -- to do-->what if not exist
	nh.getParam("logging_rate", RATE);
	ros::Rate loop_rate(RATE); 
	nh.getParam("object_name", object_name);
	pose_publishing.header.frame_id = object_name;
	
	nh.getParam("do_debug", DEBUG);
	nh.getParam("do_preview", PREVIEW);
	nh.getParam("save_images", SAVE_IMAGES);
	nh.getParam("use_prev_rvec_tvec", USE_PREV_RVEC_TVEC);
	nh.getParam("do_fcu_refinement", DO_FCU_REFINEMENT);
	
	nh.getParam("num_rhombis", NUM_RHOMBIS);
	rtvecs.resize(NUM_RHOMBIS);
	nh.getParam("self_rhombi", self_rhombi);
	nh.getParam("min_detections", min_detections);
	
	nh.getParam("DETECTION_MODE", DETECTION_MODE);
	nh.getParam("REFINEMENT_MODE", REFINEMENT_MODE);
	nh.getParam("DICTIONARY", DICTIONARY);
	nh.getParam("image_encoding", image_encoding);
	
	nh.getParam("contrast", contrast);
	nh.getParam("brightness", brightness);
	
	nh.getParam("image_savepath", image_savepath);
	if(image_savepath.empty()){ROS_WARN("Image path named %s empty. Will not save images.", image_savepath.c_str()); SAVE_IMAGES = false;}
	
	//Read geometry from file
	nh.getParam("rhombi_gmtry_path",rhombi_gmtry_path);
	if(rhombi_gmtry_path.empty()){ROS_ERROR("Rhombi geometry file %s not found. Exiting...", rhombi_gmtry_path.c_str()); return -1;}
	else{
		std::ifstream geom_outfile(rhombi_gmtry_path + ".csv");
		if (!geom_outfile.is_open()){
			ROS_INFO("Could not open geometry file");
			return -1;
		}
		else{
			std::string line = "";
			std::string delim = ";";
			while(getline(geom_outfile, line)){
				std::vector<std::string> vec;
				boost::algorithm::split(vec, line, boost::is_any_of(delim));
				std::vector<double> vec_d;
				for(uint8_t i=0; i<vec.size(); i++) vec_d.push_back(std::atof(vec[i].c_str()));
				MarkerRhombiGeometries_.RhombiCornerGeometries.push_back(vec_d);
			}
			if(DEBUG) for(int k=0; k<MarkerRhombiGeometries_.RhombiCornerGeometries.size(); k++){
				std::cout << "New geom: "<< std::endl;
				for(int m=0; m< MarkerRhombiGeometries_.RhombiCornerGeometries[k].size(); m++)
					std::cout<< MarkerRhombiGeometries_.RhombiCornerGeometries[k][m] << " " ;}
		}
	}
	
	//Generate Vector to hold the marker ID configuration for each of the Rhombis
	for(int r=0; r<NUM_RHOMBIS; r++){ //for all rhombis
		std::vector<int> single_rhombi_config;
		for(int o=0; o<MarkerRhombiGeometries_.RhombiCornerGeometries.size(); o++){ //find how many marker each rhombi has
			//assignment is a heuristic for now
			single_rhombi_config.push_back( r * MarkerRhombiGeometries_.RhombiCornerGeometries.size() + o);
		}
		MarkerRhombiGeometries_.RhombiMarkerConfigs.push_back(single_rhombi_config);
	}
	
	//Init ArUco 
	Mdetector.setDictionary(DICTIONARY, 0.f);
	Mdetector.setDetectionMode(static_cast<aruco::DetectionMode>(DETECTION_MODE),0.f);
	MarkerDetector::Params _param=Mdetector.getParameters();
	_param.setCornerRefinementMethod(static_cast<aruco::CornerRefinementMethod>(REFINEMENT_MODE));
	Mdetector.setParameters(_param);
	
	//Wait for drone to get to flight height
	ros::Subscriber tracking_start = nh.subscribe<std_msgs::Bool> ("/rhombi_det_execution", 1, &experimentHandleFeedback);
	experiment_play = true;
	while(!experiment_play){
		ros::spinOnce();
		loop_rate.sleep();
	}
	//Create subscriber to get Camera Info and calibration -- Runs only once
	sensor_msgs::CameraInfoConstPtr _CameraInfo= ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_array/cam0/camera_info", nh);
	if(_CameraInfo != NULL){
		sensor_msgs::CameraInfo cam_inf=*_CameraInfo;
		CamParam.setParams(cv::Mat(3, 3, CV_64F, &cam_inf.K[0]), cv::Mat(1, 5, CV_64F, &cam_inf.D[0]), Size(cam_inf.width, cam_inf.height));
	}
	else{
		ROS_INFO("Camera info file received is empty. Exiting...");
		return -1;
	}
	//Init acquisition 
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera_array/cam0/image_raw", 1, imageCallback);
	ros::Publisher PosePub = nh.advertise<geometry_msgs::PoseArray>("/"+ object_name +"/pose_stamped", 1);
	ROS_INFO("Starting rhombi detection");
	//LOOP START
	while(ros::ok() && experiment_play){
		if(!pose_publishing.poses.empty()){
			PosePub.publish(pose_publishing);
			pose_publishing.poses.clear();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Ending rhombi detection");
	return 0;
}
