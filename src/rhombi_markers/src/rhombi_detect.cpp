#include "rhombi_detect.h"

using namespace std;
using namespace aruco;	//ARUCO3
using namespace cv;

bool experiment_play=false;

MarkerDetector Mdetector;
aruco::CameraParameters CamParam;

int frameIDS = 0;
int frameIDS_final = 0;

marker_geom MarkerRhombiGeometries_;
std::vector<rhombi_detect> rtvecs;

//For plotting the ArUco axes do not delete. Faster execution
std::vector< cv::Point2f > imagePointsA;
std::vector< cv::Point3f > axisPoints;
Mat imagePointsVelocities = (cv::Mat_<double>(15,8) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0);

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

void solvePNP_NE(cv::Mat srcImg){
	// Copy image to visualize Axis and/or save
	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	if(contrast != 1.0 || brightness != 0.0){
		srcImg.convertTo(srcImg,-1,contrast,brightness);
	}
	//Find all markers in frame
	std::vector<Marker> Markers;
	std::vector<Marker> MarkersFinal;
	if(DO_CROP_IMAGE && imagePointsA.size()>0){
		//Make sure cropping limits don't exceed image limits
		int crop_x_start = imagePointsA[0].x-crop_width/2;
		int crop_y_start = imagePointsA[0].y-crop_height/2;
		if(crop_x_start<0) crop_x_start = 0;
		if(crop_y_start<0) crop_y_start = 0;
		if((crop_x_start + crop_width) > image_width ) crop_x_start = image_width - crop_width;
		if((crop_y_start + crop_height) > image_height ) crop_y_start = image_height - crop_height;
		//Crop image
		cv::Mat cropedImage = srcImg(cv::Rect(crop_x_start,crop_y_start,crop_width,crop_height));
		//Detect marker edges
		Markers = Mdetector.detect(cropedImage);
		for(auto m : Markers){
			int id = m.id;
			std::vector<cv::Point2f> corners;
			for(uint8_t c=0; c<4; c++){
				corners.push_back(cv::Point2f(m[c].x + crop_x_start, m[c].y + crop_y_start));
			}
			MarkersFinal.push_back(aruco::Marker(corners,id));
		}
		imagePointsA.empty();
		//imshow("Cropped",cropedImage);	cv::waitKey(1);
	}
	else  MarkersFinal = Mdetector.detect(srcImg); 
	
	//Draw the markers
	if(MarkersFinal.size()>0){ //min detections is per Rhombi but marker detectios should also be at least that
		//Draw markers on image
		cv::Mat imageAxes;// Do not delete dummy Mat for image saving to work
		if(PREVIEW || SAVE_IMAGES){
			cv::Mat imageAxes_(srcImg.size(), CV_8UC3);
			cv::cvtColor(srcImg, imageAxes_, CV_GRAY2RGB);
			imageAxes = imageAxes_;
			for(auto m : MarkersFinal){
				m.draw(imageAxes, Scalar(0, 0, 255), 2);
			}
		}
		//For every different rhombi config check if marker exists and append
		std::vector<std::vector<aruco::Marker>> RhombiMarkerDetections(MarkerRhombiGeometries_.RhombiMarkerConfigs.size());
		for(uint8_t it1 = 0; it1 < MarkerRhombiGeometries_.RhombiMarkerConfigs.size(); it1++){
			std::vector<int> CurrentConfig = MarkerRhombiGeometries_.RhombiMarkerConfigs[it1];
			std::vector<cv::Point3d>  objectPoints;
			std::vector<cv::Point2d>  imagePoints;
			uint8_t num_detections = 0;
			for(auto k: MarkersFinal){
				std::vector<int>::iterator it = std::find(CurrentConfig.begin(), CurrentConfig.end(), k.id);
				if(it != CurrentConfig.end()){
					if(DEBUG){
						std::cout << "Found " << " Marker ID: " << k.id << " from Rhombi # " << (int) it1 << std::endl;
					}
					//-1 because marker ids start from 1 but config.csv from 0
					int index = (k.id-1)-CurrentConfig.size()*((k.id-1)/CurrentConfig.size()); 
					for(uint8_t c=0;c<4;c++){
						imagePoints.push_back(k[c]);
						objectPoints.push_back(Point3d(MarkerRhombiGeometries_.RhombiCornerGeometries[index][c*3+0],MarkerRhombiGeometries_.RhombiCornerGeometries[index][c*3+1],MarkerRhombiGeometries_.RhombiCornerGeometries[index][c*3+2]));
					}
					num_detections++;
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
					frameIDS_final++;
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
					
					if(DEBUG){
						cv::Mat to_matrix;
						cv::Vec3f to_euler_xyz;
						cv::Rodrigues(rtvecs[it1].rvec, to_matrix);
						to_euler_xyz=rotationMatrixToEulerAngles(to_matrix);
						std::cout << "Rhombi r_vec: " << to_euler_xyz[0]*180/M_PI << " " << to_euler_xyz[1]*180/M_PI << " " << to_euler_xyz[2]*180/M_PI << " " << std::endl;
						std::cout << "Rhombi t_vec: " << rtvecs[it1].tvec << std::endl << std::endl;
					}
					//Prepare image for preview/saving or cropping techniques
					if(PREVIEW || SAVE_IMAGES || DO_CROP_IMAGE){
						cv::projectPoints(axisPoints, rtvecs[it1].rvec, rtvecs[it1].tvec, CamParam.CameraMatrix, CamParam.Distorsion, imagePointsA, imagePointsVelocities);
					}
					if(PREVIEW || SAVE_IMAGES){
						// draw axis lines
						if(inRange(0,CamParam.CamSize.width,imagePointsA[0].x) && inRange(0,CamParam.CamSize.height,imagePointsA[0].y) &&
							inRange(0,CamParam.CamSize.width,imagePointsA[1].x) && inRange(0,CamParam.CamSize.height,imagePointsA[1].y) &&
							inRange(0,CamParam.CamSize.width,imagePointsA[2].x) && inRange(0,CamParam.CamSize.height,imagePointsA[2].y) &&
							inRange(0,CamParam.CamSize.width,imagePointsA[3].x) && inRange(0,CamParam.CamSize.height,imagePointsA[3].y)){
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
		cv::Mat image2;
		cv::resize(imageAxes,image2,cv::Size(1024,768));
		if(PREVIEW)	{imshow("RhombiAxes",image2);	cv::waitKey(1);}
		if(SAVE_IMAGES)	cv::imwrite(image_savepath + std::to_string(pose_publishing.header.seq) + ".bmp",imageAxes);
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
			if(frameIDS == 0) frameIDS = frameIDS_final = msg->header.seq;
			//Prepare publishing message -- Time in seconds since EPOCH - post processing required to get elapsed
			pose_publishing.header.seq = msg->header.seq;
			pose_publishing.header.stamp = msg->header.stamp;
			pose_publishing.header.frame_id = object_name + std::to_string(pose_publishing.header.seq);
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
	nh.getParam("detection_rate", RATE);
	ros::Rate loop_rate(RATE); 
	nh.getParam("object_name", object_name);
	pose_publishing.header.frame_id = object_name;
	
	nh.getParam("do_debug", DEBUG);
	nh.getParam("do_preview", PREVIEW);
	nh.getParam("save_images", SAVE_IMAGES);
	nh.getParam("use_prev_rvec_tvec", USE_PREV_RVEC_TVEC);
	nh.getParam("do_fcu_refinement", DO_FCU_REFINEMENT);
	nh.getParam("do_crop_image", DO_CROP_IMAGE);
	nh.getParam("crop_width", crop_width);
	nh.getParam("crop_height", crop_height);
	
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
	axisPoints.push_back(Point3f(0, 0, 0));axisPoints.push_back(Point3f(0.01 * 7.5f, 0, 0));
	axisPoints.push_back(Point3f(0, 0.01 * 7.5f, 0));axisPoints.push_back(Point3f(0, 0, 0.01 * 7.5f));
	
	//Wait for drone to get to flight height
	ros::Subscriber tracking_start = nh.subscribe<std_msgs::Bool> ("/experiment_execution", 1, &experimentHandleFeedback);
	experiment_play = true;
	while(!experiment_play){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
//Create subscriber to get Camera Info and calibration or load from file
// 	sensor_msgs::CameraInfoConstPtr _CameraInfo= ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/object_name/flir/camera_info", nh);
// 	if(_CameraInfo != NULL){
// 		sensor_msgs::CameraInfo cam_inf=*_CameraInfo;
// 		CamParam.setParams(cv::Mat(3, 3, CV_64F, &cam_inf.K[0]), cv::Mat(1, 5, CV_64F, &cam_inf.D[0]), Size(cam_inf.width, cam_inf.height));
// 	}
// 	else{
// 		ROS_INFO("Camera info file received is empty. Exiting...");
// 		return -1;
// 	}
	bool intrinsics_list_provided = false;
	std::vector<double> intrinsics;
    XmlRpc::XmlRpcValue intrinsics_list;
    if (nh.getParam("intrinsic_coeffs", intrinsics_list)) {
        ROS_INFO("  Camera Intrinsic Paramters:");for (int i=0; i<intrinsics_list.size(); i++){
            cv::String intrinsics_str="";
            for (int j=0; j<intrinsics_list[i].size(); j++){
                ROS_ASSERT_MSG(intrinsics_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                intrinsics.push_back(static_cast<double>(intrinsics_list[i][j]));
                intrinsics_str = intrinsics_str +to_string(intrinsics[j])+" ";
            }
			//cv::Mat cameraIntrincics_ = cv::Mat(3, 3, CV_64F, &intrinsics[0]);
			//cameraIntrincics_.convertTo(cameraIntrincics,CV_64FC1);
            ROS_INFO_STREAM("   "<< intrinsics_str );
            intrinsics_list_provided=true;
        }
    }
    bool distort_list_provided = false;
	std::vector<double> distort;
    XmlRpc::XmlRpcValue distort_list;
    if (nh.getParam("distortion_coeffs", distort_list)) {
        ROS_INFO("  Camera Distortion Parameters:");
        for (int i=0; i<distort_list.size(); i++){
            cv::String distort_str="";
            for (int j=0; j<distort_list[i].size(); j++){
                ROS_ASSERT_MSG(distort_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                distort.push_back(static_cast<double>(distort_list[i][j]));
                distort_str = distort_str +to_string(distort[j])+" ";
            }
            //cv::Mat distCoeffs_ = cv::Mat(1, 5, CV_64F, &distort[0]);
			//distCoeffs_.convertTo(distortionCoeffs,CV_64FC1);
            ROS_INFO_STREAM("   "<< distort_str );
            distort_list_provided = true;
        }
    }
    
    nh.getParam("image_height", image_height);
	nh.getParam("image_width", image_width);
	if(image_height < crop_height){ROS_INFO("Image_height<Cropping_height. Resetting cropping height"); crop_height = image_height;}
	if(image_width < crop_width){ROS_INFO("Image_width<Cropping_width. Resetting cropping width"); crop_width = image_width;}
	
    if(distort_list_provided && intrinsics_list_provided)
    CamParam.setParams(cv::Mat(3, 3, CV_64FC1, &intrinsics[0]), cv::Mat(1, 5, CV_64FC1, &distort[0]), cv::Size(image_width, image_height));
	
	//Init acquisition 
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/" + object_name + "/flir/image_raw", 1, imageCallback);
	ros::Publisher PosePub = nh.advertise<geometry_msgs::PoseArray>("/"+ object_name +"/pose_stamped", 1);
	ROS_INFO("Starting rhombi detection");
	
	// Run example
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	
	//LOOP START
	while(ros::ok() && experiment_play){
		if(!pose_publishing.poses.empty()){
			PosePub.publish(pose_publishing);
			pose_publishing.poses.clear();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Processed " << frameIDS_final - frameIDS << " images in " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [µs]." << std::endl;
	std::cout << "Rate was: " << (frameIDS_final- frameIDS)*1E6/ std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " Hz" <<std::endl;
	ROS_INFO("Ending rhombi detection");
	return 0;
}
