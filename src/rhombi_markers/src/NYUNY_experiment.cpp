#include "NYUNY_experiment.h"
#include <termios.h>
#include <fcntl.h>

bool experiment_play=false;

int frameIDS = 0;

//Keyboard function used to terminate the program on any keyboard button hit
int kbhit(void)
{
	struct termios oldt, newt;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	int ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)	return 1;
	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) //(const sensor_msgs::ImageConstPtr& msg, int image360part)
{
	try
	{
		cv::Mat srcImg = cv_bridge::toCvShare(msg, image_encoding)->image;
		if(!srcImg.empty()){
			if(DEBUG) ROS_INFO("Got new image!");
			if(contrast != 1.0 || brightness != 0.0){
				srcImg.convertTo(srcImg,-1,contrast,brightness);
			}
			auto markers=MDetector.detect(srcImg);//0.05 is the marker size
			for(auto m : markers){
				m.draw(srcImg, cv::Scalar(0, 0, 255), 2);
			}
			if (MMTracker.isValid() && MMTracker.estimatePose(markers)){
				cv::Mat1d rvec = MMTracker.getRvec();
				cv::Mat1d tvec = MMTracker.getTvec();
				if(DEBUG)	std::cout<< rvec << " " << tvec << std::endl;
				//Prepare image for preview/saving
				if(PREVIEW || SAVE_IMAGES){
					aruco::CvDrawingUtils::draw3dAxis(srcImg, CamParam, rvec, tvec, 0.5);
				}
				MarkerMapPose.header.seq = msg->header.seq;
				MarkerMapPose.header.stamp = msg-> header.stamp;
				cv::Mat R2(3, 3, CV_64FC1);
				cv::Rodrigues(rvec, R2);
				Eigen::Matrix3d eigMat2;
				cv::cv2eigen(R2, eigMat2);
				Eigen::Quaterniond quat2(eigMat2);
				tf::quaternionEigenToMsg(quat2 , MarkerMapPose.pose.orientation);
				MarkerMapPose.pose.position.x = tvec(0,0);
				MarkerMapPose.pose.position.y = tvec(0,1);
				MarkerMapPose.pose.position.z = tvec(0,2);
				PosePub.publish(MarkerMapPose);
			}
			if(PREVIEW)	{imshow("FLIR_image",srcImg);	cv::waitKey(1);}
			if(SAVE_IMAGES)	cv::imwrite(image_savepath + std::to_string(MarkerMapPose.header.seq) + ".bmp", srcImg);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_WARN("Could not convert from %s",image_encoding.c_str());
	}
}


void experimentHandleFeedback(std_msgs::Bool thehandle ){
	experiment_play = thehandle.data;
}

//MAIN
int main(int argc, char **argv)
{  
	//Init ROS
	ros::init(argc, argv, "rhombi_detect");
	ros::NodeHandle nh("~");
	//Get PARAMS from server -- to do-->what if not exist
	nh.getParam("rhombi_logging_rate", RATE);
	ros::Rate loop_rate(RATE); 
	nh.getParam("object_name", object_name);
	MarkerMapPose.header.frame_id = object_name;
	
	nh.getParam("do_debug", DEBUG);
	nh.getParam("do_preview", PREVIEW);
	nh.getParam("save_images", SAVE_IMAGES);
	
	nh.getParam("image_encoding", image_encoding);
	nh.getParam("DETECTION_MODE", DETECTION_MODE);
	nh.getParam("REFINEMENT_MODE", REFINEMENT_MODE);
	nh.getParam("DICTIONARY", DICTIONARY);
	
	nh.getParam("contrast", contrast);
	nh.getParam("brightness", brightness);
	
	nh.getParam("cam_param_path",cam_param_path);
	if(cam_param_path.empty()){ROS_ERROR("Cam param %s empty. Exiting...", cam_param_path.c_str()); return -1;}
	nh.getParam("mmap_path",mmap_path);
	if(mmap_path.empty()){ROS_ERROR("Marker map path %s empty. Exiting...", mmap_path.c_str()); return -1;}
	nh.getParam("image_savepath", image_savepath);
	if(image_savepath.empty()){ROS_WARN("Image path named %s empty. Will not save images.", image_savepath.c_str()); SAVE_IMAGES = false;}
	
	//Get Camera calibration
	CamParam.readFromXMLFile(cam_param_path);
	std::cout<< CamParam.CameraMatrix <<std::endl;
	//return 0;
	
	//Get MarkerMap
	aruco::MarkerMap mmap;
	mmap.readFromFile(mmap_path);
	MMTracker.setParams(CamParam,mmap);
	
	//Init ArUco 
	MDetector.setDictionary(DICTIONARY, 0.f);
	MDetector.setDetectionMode(static_cast<aruco::DetectionMode>(DETECTION_MODE),0.f);
	aruco::MarkerDetector::Params _param=MDetector.getParameters();
	_param.setCornerRefinementMethod(static_cast<aruco::CornerRefinementMethod>(REFINEMENT_MODE));
	MDetector.setParameters(_param);
	
	//Wait for drone to get to flight height
	ros::Subscriber tracking_start = nh.subscribe<std_msgs::Bool> ("/grabbing_control", 1, &experimentHandleFeedback);
	while(!experiment_play){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	//Init acquisition 
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/" + object_name + "/flir/image_raw", 1, imageCallback);
	PosePub = nh.advertise<geometry_msgs::PoseStamped>("/"+ object_name + "/aruco_grid_board/pose_stamped", 1);
	ROS_INFO("Starting image grab...");
	
	//LOOP START
	while(ros::ok() && !kbhit() && experiment_play){
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Ending image grab...");
	return 0;
}
