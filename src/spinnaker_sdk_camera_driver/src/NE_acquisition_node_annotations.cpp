#include "spinnaker_sdk_camera_driver/NYUNY_v1.h"

//=============================================================================
// Copyright (c) 2001-2019 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR 
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
 *  @example Acquisition.cpp
 *
 *  @brief Acquisition.cpp shows how to acquire images. It relies on
 *  information provided in the Enumeration example. Also, check out the
 *  ExceptionHandling and NodeMapInfo examples if you haven't already.
 *  ExceptionHandling shows the handling of standard and Spinnaker exceptions
 *  while NodeMapInfo explores retrieving information from various node types.
 *
 *  This example touches on the preparation and cleanup of a camera just before
 *  and just after the acquisition of images. Image retrieval and conversion,
 *  grabbing image data, and saving images are all covered as well.
 *
 *  Once comfortable with Acquisition, we suggest checking out
 *  AcquisitionMultipleCamera, NodeMapCallback, or SaveToAvi.
 *  AcquisitionMultipleCamera demonstrates simultaneously acquiring images from
 *  a number of cameras, NodeMapCallback serves as a good introduction to
 *  programming with callbacks and events, and SaveToAvi exhibits video creation.
 */

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

std::vector<geometry_msgs::PoseStamped>  TVPose; 
std::vector<geometry_msgs::PoseStamped>  CCDPose;
tf::Transform Camera_Axes_Flip, Vicon_TV_pose_Correction, Fixation_mat, Camera_2_WorldFrame;
cv::Mat cameraIntrincics(3,3,CV_64FC1), distortionCoeffs(1,5,CV_64FC1);

//const float pixel_size = 0.3715e-3;
const float pixel_size = 0.3715e-3;
const double TV_width = 3840  * pixel_size;
const double TV_height = 2160 * pixel_size;
const float markerSizePixel = 3;
const cv::Point2d mSPix(markerSizePixel,markerSizePixel);

std::vector<tf::Vector3> TV_corners;

const int meas_to_hold = 100;

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int DisableHeartbeat(INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
	cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;
	
	//
	// Write to boolean node controlling the camera's heartbeat
	//
	// *** NOTES ***
	// This applies only to GEV cameras and only applies when in DEBUG mode.
	// GEV cameras have a heartbeat built in, but when debugging applications the
	// camera may time out due to its heartbeat. Disabling the heartbeat prevents
	// this timeout from occurring, enabling us to continue with any necessary debugging.
	// This procedure does not affect other types of cameras and will prematurely exit
	// if it determines the device in question is not a GEV camera.
	//
	// *** LATER ***
	// Since we only disable the heartbeat on GEV cameras during debug mode, it is better
	// to power cycle the camera after debugging. A power cycle will reset the camera
	// to its default settings.
	//
	CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
	if (!IsAvailable(ptrDeviceType) || !IsReadable(ptrDeviceType))
	{
		cout << "Error with reading the device's type. Aborting..." << endl << endl;
		return -1;
	}
	else
	{
		if (ptrDeviceType->GetIntValue() == DeviceType_GigEVision)
		{
			cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
			CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
			if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
			{
				cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..."
				<< endl
				<< endl;
			}
			else
			{
				ptrDeviceHeartbeat->SetValue(true);
				cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
				cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
			}
		}
		else
		{
			cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
		}
	}
	return 0;
}
#endif

void experimentHandleFeedback(std_msgs::Bool thehandle ){
	experiment_play = thehandle.data;
	std::cout<< " " << std::endl;
}

void CCDPoseFeedback(geometry_msgs::PoseStampedConstPtr CCDPose_){
	//Hold only latest meas_to_hold elements
	if(CCDPose.size()>=meas_to_hold) CCDPose.erase(CCDPose.begin());
	geometry_msgs::PoseStamped tmp_pose = *CCDPose_;
	//Account for Vicon server drift in Windows
	tmp_pose.header.stamp.fromSec(tmp_pose.header.stamp.toSec()-8*3600.0);
	CCDPose.push_back( tmp_pose);
}

void MonitorPoseFeedback(geometry_msgs::PoseStampedConstPtr MonitorPose_){
	//Hold only latest meas_to_hold elements
	if(TVPose.size()>=meas_to_hold) TVPose.erase(TVPose.begin());
	geometry_msgs::PoseStamped tmp_pose = *MonitorPose_;
	//Account for Vicon server drift in Windows
	tmp_pose.header.stamp.fromSec(tmp_pose.header.stamp.toSec()-8*3600.0);
	TVPose.push_back( tmp_pose);
}


// This function acquires and saves 10 images from a device.
int AcquireImages(CameraPtr pCam, INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
	int result = 0;
	int imageCnt = 0;
	cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;
	
	try
	{
		//
		// Set acquisition mode to continuous
		//
		// *** NOTES ***
		// Because the example acquires and saves 10 images, setting acquisition
		// mode to continuous lets the example finish. If set to single frame
		// or multiframe (at a lower number of images), the example would just
		// hang. This would happen because the example has been written to
		// acquire 10 images while the camera would have been programmed to
		// retrieve less than that.
		//
		// Setting the value of an enumeration node is slightly more complicated
		// than other node types. Two nodes must be retrieved: first, the
		// enumeration node is retrieved from the nodemap; and second, the entry
		// node is retrieved from the enumeration node. The integer value of the
		// entry node is then set as the new value of the enumeration node.
		//
		// Notice that both the enumeration and the entry nodes are checked for
		// availability and readability/writability. Enumeration nodes are
		// generally readable and writable whereas their entry nodes are only
		// ever readable.
		//
		// Retrieve enumeration node from nodemap
		CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
		if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
		{
			cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
			return -1;
		}
		
		// Retrieve entry node from enumeration node
		CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
		if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
		{
			cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
			return -1;
		}
		
		// Retrieve integer value from entry node
		const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
		
		// Set integer value from entry node as new value of enumeration node
		ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
		
		cout << "Acquisition mode set to continuous..." << endl;
		
		#ifdef _DEBUG
		cout << endl << endl << "*** DEBUG ***" << endl << endl;
		
		// If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
		if (DisableHeartbeat(nodeMap, nodeMapTLDevice) != 0)
		{
			return -1;
		}
		
		cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
		#endif
		
		gcstring deviceSerialNumber("");
		CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
		if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
		{
			deviceSerialNumber = ptrStringSerial->GetValue();
			
			cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
		}
		
		// Retrieve Stream Parameters device nodemap
        Spinnaker::GenApi::INodeMap& sNodeMap = pCam->GetTLStreamNodeMap();

        // Retrieve Buffer Handling Mode Information
        CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
        if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
        {
            cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << endl << endl;
            return -1;
        }

        CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
        if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
        {
            cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << endl << endl;
            return -1;
        }

        // Set stream buffer Count Mode to manual
        CEnumerationPtr ptrStreamBufferCountMode = sNodeMap.GetNode("StreamBufferCountMode");
        if (!IsAvailable(ptrStreamBufferCountMode) || !IsWritable(ptrStreamBufferCountMode))
        {
            cout << "Unable to set Buffer Count Mode (node retrieval). Aborting..." << endl << endl;
            return -1;
        }

        CEnumEntryPtr ptrStreamBufferCountModeManual = ptrStreamBufferCountMode->GetEntryByName("Manual");
        if (!IsAvailable(ptrStreamBufferCountModeManual) || !IsReadable(ptrStreamBufferCountModeManual))
        {
            cout << "Unable to set Buffer Count Mode entry (Entry retrieval). Aborting..." << endl << endl;
            return -1;
        }

        ptrStreamBufferCountMode->SetIntValue(ptrStreamBufferCountModeManual->GetValue());

        cout << "Stream Buffer Count Mode set to manual..." << endl;

        // Retrieve and modify Stream Buffer Count
        CIntegerPtr ptrBufferCount = sNodeMap.GetNode("StreamBufferCountManual");
        if (!IsAvailable(ptrBufferCount) || !IsWritable(ptrBufferCount))
        {
            cout << "Unable to set Buffer Count (Integer node retrieval). Aborting..." << endl << endl;
            return -1;
        }

        // Display Buffer Info
        cout << endl << "Default Buffer Handling Mode: " << ptrHandlingModeEntry->GetDisplayName() << endl;
        cout << "Default Buffer Count: " << ptrBufferCount->GetValue() << endl;
        cout << "Maximum Buffer Count: " << ptrBufferCount->GetMax() << endl;

        ptrBufferCount->SetValue(2);
		
		ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
		ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
		cout << endl
				<< endl
				<< "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << endl;

        cout << "Buffer count now set to: " << ptrBufferCount->GetValue() << endl;
		
		//
		// Begin acquiring images
		//
		// *** NOTES ***
		// What happens when the camera begins acquiring images depends on the
		// acquisition mode. Single frame captures only a single image, multi
		// frame captures a set number of images, and continuous captures a
		// continuous stream of images. Because the example calls for the
		// retrieval of 10 images, continuous mode has been set.
		//
		// *** LATER ***
		// Image acquisition must be ended when no more images are needed.
		//
		std::cout << "Acquiring images..." << std::endl;
		pCam->TimestampReset();
		ros::Time image_time= ros::Time::now();
		pCam->BeginAcquisition();
		//
		// Retrieve device serial number for filename
		//
		// *** NOTES ***
		// The device serial number is retrieved in order to keep cameras from
		// overwriting one another. Grabbing image IDs could also accomplish
		// this.
		//
		
		//while(ros::ok() && !kbhit() && experiment_play && imageCnt<1000){
		while(ros::ok() && !kbhit() && experiment_play){
			try
			{
				// Retrieve next received image
				//
				// *** NOTES ***
				// Capturing an image houses images on the camera buffer. Trying
				// to capture an image that does not exist will hang the camera.
				//
				// *** LATER ***
				// Once an image from the buffer is saved and/or no longer
				// needed, the image must be released in order to keep the
				// buffer from filling up.
				//
				ImagePtr pResultImage = pCam->GetNextImage(50);
				//
				// Ensure image completion
				//
				// *** NOTES ***
				// Images can easily be checked for completion. This should be
				// done whenever a complete image is expected or required.
				// Further, check image status for a little more insight into
				// why an image is incomplete.
				//
				if (pResultImage->IsIncomplete() || pResultImage->GetImageStatus() != IMAGE_NO_ERROR)
				{
					// Retrieve and print the image status description
					cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
					<< "..." << endl
					<< endl;
				}
				else
				{
					//
					// Print image information; height and width recorded in pixels
					//
					// *** NOTES ***
					// Images have quite a bit of available metadata including
					// things such as CRC, image status, and offset values, to
					// name a few.
					//
					//const size_t width = pResultImage->GetWidth();
					//const size_t height = pResultImage->GetHeight();
					if(DEBUG) std::cout << "Grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << std::endl;
					//
					// Convert image to ....
					//
					// *** NOTES ***
					// Images can be converted between pixel formats by using
					// the appropriate enumeration value. Unlike the original
					// image, the converted one does not need to be released as
					// it does not affect the camera buffer.
					//
					// When converting images, color processing algorithm is an
					// optional parameter.
					//
					//ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
					//
					ImagePtr convertedImage;
					//For now only mono8 and bgr8 encodings
					//5ms
					if (image_encoding != "mono8") convertedImage = pResultImage->Convert(PixelFormat_BGR8, IPP); //, NEAREST_NEIGHBOR);
					else	convertedImage = pResultImage->Convert(PixelFormat_Mono8, IPP); //, NEAREST_NEIGHBOR);
					unsigned int XPadding = convertedImage->GetXPadding();
					unsigned int YPadding = convertedImage->GetYPadding();
					unsigned int rowsize = convertedImage->GetWidth();
					unsigned int colsize = convertedImage->GetHeight();
					
					//Image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
					cv::Mat srcImg;
					if (image_encoding != "mono8")	srcImg = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
					else	srcImg = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride());
					ROS_INFO("1");
					//Image manipulation here////////////////////////////////////////////
					tf::Transform CCD_tf;
					tf::poseMsgToTF(CCDPose[CCDPose.size()-1].pose, CCD_tf);
					CCD_tf *= Camera_Axes_Flip;
					tf::Transform Monitor_tf;
					tf::poseMsgToTF(TVPose[TVPose.size()-1].pose, Monitor_tf);
					Monitor_tf *= Vicon_TV_pose_Correction;
					tf::Transform final = CCD_tf.inverseTimes(Monitor_tf);
					final = Fixation_mat * final; //Careful left multiply!
						double yaw, pitch, roll;
						final.getBasis().getRPY(roll, pitch, yaw);
						tf::Quaternion q = final.getRotation();
						tf::Vector3 v = final.getOrigin();
						std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
						std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
								<< q.getZ() << ", " << q.getW() << "]" << std::endl
								<< "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
								<< "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
					ROS_INFO("1a");
					//cv::Mat undImg;
					//cv::undistort(srcImg, undImg, cameraIntrincics, distortionCoeffs);
					std::vector<cv::Point3d>  objectPoint;
					std::vector<cv::Point2d>  imagePoints;
					tf::Vector3 tmp_point_location;
					for(uint8_t i=0; i< TV_corners.size(); i++){
						tmp_point_location = final.getBasis() * TV_corners[i] + final.getOrigin();
						std::cout << tmp_point_location.getX() << " " << tmp_point_location.getY() << " " 
									<< tmp_point_location.getZ() << std::endl;
						objectPoint.push_back(cv::Point3f(tmp_point_location.getX(),
													  tmp_point_location.getY(),
													  tmp_point_location.getZ()));
// 						tf::Vector3 tmp_point_location_norm(tmp_point_location.getX()/tmp_point_location.getZ(),
// 													  tmp_point_location.getY()/tmp_point_location.getZ(),
// 													  1.0);
// 						imagePoints.push_back(cv::Point2d(cameraIntrincics.at<double>(0,0)*tmp_point_location_norm.getX()+
// 												cameraIntrincics.at<double>(0,1)*tmp_point_location_norm.getY()+ cameraIntrincics.at<double>(0,2),
// 															cameraIntrincics.at<double>(1,1)*tmp_point_location_norm.getY()+ cameraIntrincics.at<double>(1,2)));
					}
					ROS_INFO("2");
					cv::projectPoints(objectPoint, cv::Mat::zeros(3,1,CV_32FC1), cv::Mat::zeros(3,1,CV_32FC1), cameraIntrincics, distortionCoeffs,  imagePoints);
					ROS_INFO("3");
					for(uint8_t i=0; i<imagePoints.size(); i++){
						cv::Point2d rect_top_left = imagePoints[i] - mSPix;
						cv::Point2d rect_low_right = imagePoints[i] + mSPix;
						if(rect_top_left.x > 0 && rect_top_left.y > 0 && rect_low_right.x < pResultImage->GetWidth() && rect_low_right.y < pResultImage->GetHeight())
						//cv::rectangle(undImg,rect_top_left ,rect_low_right, cv::Scalar(0, 255, 0, 255));
						cv::rectangle(srcImg,rect_top_left ,rect_low_right, cv::Scalar(0, 255, 0, 255));
					}
					
					//////////////////////////////////////////////////////////////////////
					
					if(PREVIEW)	{cv::imshow("FLIR_image",srcImg);	cv::waitKey(1);}
					//if(PREVIEW)	{cv::imshow("FLIR_image",undImg);	cv::waitKey(1);}
				
					//if to_ROS?
					//5ms
					if(image_to_ros){
						sensor_msgs::ImagePtr img_msgs;
						cam_info_Headers.frame_id = std::to_string(imageCnt);
						cam_info_Headers.seq = (uint32_t) imageCnt;
						cam_info_Headers.stamp.fromNSec(image_time.toNSec() + pResultImage->GetTimeStamp());
						img_msgs = cv_bridge::CvImage(cam_info_Headers, image_encoding, srcImg).toImageMsg();
						camera_image_pubs.publish(img_msgs);
					}
					
					// Save image
					//12ms
					if(SAVE_IMAGES){ 
						std::string filename = image_savepath + std::to_string(imageCnt) + ".bmp";
						convertedImage->Save(filename.c_str());// or cv::imwrite(filename.c_str(), srcImg);
					}
					//ROS_INFO("%d", imageCnt);
					imageCnt++;
				}
				//
				// Release image
				//
				// *** NOTES ***
				// Images retrieved directly from the camera (i.e. non-converted
				// images) need to be released in order to keep from filling the
				// buffer.
				pResultImage->Release();
			}
			catch (Spinnaker::Exception& e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}
			ros::spinOnce();
			ros::Duration(1.0/RATE).sleep();
		}
		//
		// End acquisition
		//
		// *** NOTES ***
		// Ending acquisition appropriately helps ensure that devices clean up
		// properly and do not need to be power-cycled to maintain integrity.
		pCam->EndAcquisition();
	}
	catch (Spinnaker::Exception& e)
	{
		cout << "Error: " << e.what() << endl;
		return -1;
	}
	//return result;
	return imageCnt;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap)
{
	int result = 0;
	cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;
	
	try
	{
		FeatureList_t features;
		const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
		if (IsAvailable(category) && IsReadable(category))
		{
			category->GetFeatures(features);
			
			for (auto it = features.begin(); it != features.end(); ++it)
			{
				const CNodePtr pfeatureNode = *it;
				cout << pfeatureNode->GetName() << " : ";
				CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
				cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
				cout << endl;
			}
		}
		else
		{
			cout << "Device control information not available." << endl;
		}
	}
	catch (Spinnaker::Exception& e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	
	return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam)
{
	int result;
	
	try
	{
		// Retrieve TL device nodemap and print device information
		INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
		
		result = PrintDeviceInfo(nodeMapTLDevice);
		
		// Initialize camera
		pCam->Init();
		
		// Retrieve GenICam nodemap
		INodeMap& nodeMap = pCam->GetNodeMap();
		
		// Acquire images
		result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice);
		
		// Deinitialize camera
		pCam->DeInit();
	}
	catch (Spinnaker::Exception& e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	
	return result;
}

// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int argc, char** argv)
{
	// Initializing the ros node
	ros::init(argc, argv, "FLIR_acquisition_rate");
	//Init ROS
	ros::NodeHandle nh("~");
	
	//Get PARAMS from server -- to do-->what if not exist
	nh.getParam("FLIR_acquisition_rate", RATE);
	nh.getParam("object_name", object_name);
	image_transport::ImageTransport it_(nh);
	camera_image_pubs = it_.advertise("/" + object_name + "/flir/image_raw", 1);
	
	//Initialization - Publishers Subscribers
	ros::Subscriber CCD_Pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ object_name + "/pose", 1, &CCDPoseFeedback);
	ros::Subscriber TV_Pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Monitor/pose", 1, &MonitorPoseFeedback);
	//Wait for all callbacks to run -- Hard constraint
	while(CCD_Pos_sub.getNumPublishers()==0 || TV_Pos_sub.getNumPublishers()==0){
		ros::spinOnce();
		ros::Duration(1.0/RATE).sleep();
	}
	//Initialize transformations///////////////////////////////
	tf::Quaternion Camera_Axes_Flip_quat(-0.5,0.5,-0.5, 0.5);
	Camera_Axes_Flip.setRotation(Camera_Axes_Flip_quat);
	tf::Vector3 Camera_Axes_Flip_trans(0,0,0);
	Camera_Axes_Flip.setOrigin(Camera_Axes_Flip_trans);
	
	tf::Quaternion Vicon_TV_pose_Correction_quat(0.5,0.5,0.5,0.5);
	Vicon_TV_pose_Correction.setRotation(Vicon_TV_pose_Correction_quat);
	tf::Vector3 Vicon_TV_pose_Correction_trans(-8.25E-3, 0, 0);
	Vicon_TV_pose_Correction.setOrigin(Vicon_TV_pose_Correction_trans);
	
	tf::Quaternion Fixation_mat_quat;
	Fixation_mat_quat.setRPY(deg2rad(-0.25),deg2rad(0.25),0.0);
	Fixation_mat_quat.setRPY(0.0,0.0,0.0);
	Fixation_mat.setRotation(Fixation_mat_quat);
	tf::Vector3 Fixation_mat_trans(0,0,0);
	Fixation_mat.setOrigin(Fixation_mat_trans);
	
	TV_corners.push_back(tf::Vector3(-TV_width/2, TV_height/2, 0.0));
	TV_corners.push_back(tf::Vector3( TV_width/2, TV_height/2, 0.0));
	TV_corners.push_back(tf::Vector3( TV_width/2,-TV_height/2, 0.0));
	TV_corners.push_back(tf::Vector3(-TV_width/2,-TV_height/2, 0.0));
	
	bool intrinsics_list_provided = false;
    XmlRpc::XmlRpcValue intrinsics_list;
    if (nh.getParam("intrinsic_coeffs", intrinsics_list)) {
        ROS_INFO("  Camera Intrinsic Paramters:");for (int i=0; i<intrinsics_list.size(); i++){
            std::vector<double> intrinsics;
            cv::String intrinsics_str="";
            for (int j=0; j<intrinsics_list[i].size(); j++){
                ROS_ASSERT_MSG(intrinsics_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                intrinsics.push_back(static_cast<double>(intrinsics_list[i][j]));
                intrinsics_str = intrinsics_str +to_string(intrinsics[j])+" ";
				cv::Mat cameraIntrincics_ = cv::Mat(3, 3, CV_64F, &intrinsics[0]);
				cameraIntrincics_.convertTo(cameraIntrincics,CV_64FC1);
            }
            ROS_INFO_STREAM("   "<< intrinsics_str );
            intrinsics_list_provided=true;
        }
    }
    bool distort_list_provided = false;
    XmlRpc::XmlRpcValue distort_list;

    if (nh.getParam("distortion_coeffs", distort_list)) {
        ROS_INFO("  Camera Distortion Parameters:");
        for (int i=0; i<distort_list.size(); i++){
            std::vector<double> distort;
            cv::String distort_str="";
            for (int j=0; j<distort_list[i].size(); j++){
                ROS_ASSERT_MSG(distort_list[i][j].getType()== XmlRpc::XmlRpcValue::TypeDouble,"Make sure all numbers are entered as doubles eg. 0.0 or 1.1");
                distort.push_back(static_cast<double>(distort_list[i][j]));
                distort_str = distort_str +to_string(distort[j])+" ";
            }
            cv::Mat distCoeffs_ = cv::Mat(1, 5, CV_64F, &distort[0]);
			distCoeffs_.convertTo(distortionCoeffs,CV_64FC1);
            ROS_INFO_STREAM("   "<< distort_str );
            distort_list_provided = true;
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////
	
	nh.getParam("do_debug", DEBUG);
	nh.getParam("do_preview", PREVIEW);
	nh.getParam("save_images", SAVE_IMAGES);
	
	nh.getParam("image_encoding", image_encoding);
	
	nh.getParam("image_to_ros", image_to_ros);
	
	nh.getParam("image_savepath", image_savepath);
	if(SAVE_IMAGES){
		if(image_savepath.empty()){ROS_WARN("Image path named %s empty. Will not save images.", image_savepath.c_str()); SAVE_IMAGES = false;}
		else{
			// Since this application saves images in the current folder
			// we must ensure that we have permission to write to this folder.
			// If we do not have permission, fail right away.
			std::string check_savepath = image_savepath + "text.txt";
			FILE* tempFile = fopen(check_savepath.c_str(), "w+");
			if (tempFile == nullptr)
			{
				cout << "Failed to create file in current folder.  Please check "
				"permissions."
				<< endl;
				cout << "Press Enter to exit..." << endl;
				getchar();
				return -1;
			}
			fclose(tempFile);
			remove(check_savepath.c_str());
		}
	}
	
	// Retrieve singleton reference to system object
	SystemPtr system = System::GetInstance();
	
	// Retrieve list of cameras from the system
	CameraList camList = system->GetCameras();
	
	const unsigned int numCameras = camList.GetSize();
	
	cout << "Number of cameras detected: " << numCameras << endl << endl;
	
	// Finish if there are no cameras
	if (numCameras == 0)
	{
		// Clear camera list before releasing system
		camList.Clear();
		
		// Release system
		system->ReleaseInstance();
		
		cout << "Not enough cameras!" << endl;
		return -1;
	}
	
	// Create shared pointer to camera
	//
	// *** NOTES ***
	// The CameraPtr object is a shared pointer, and will generally clean itself
	// up upon exiting its scope. However, if a shared pointer is created in the
	// same scope that a system object is explicitly released (i.e. this scope),
	// the reference to the shared point must be broken manually.
	//
	// *** LATER ***
	// Shared pointers can be terminated manually by assigning them to nullptr.
	// This keeps releasing the system from throwing an exception.
	//
	CameraPtr pCam = nullptr;

	//Wait for drone to get to flight height
	ros::Subscriber tracking_start = nh.subscribe<std_msgs::Bool> ("/grabbing_control", 1, &experimentHandleFeedback);
	experiment_play = false;
	while(ros::ok() && !kbhit() && !experiment_play){
		ros::spinOnce();
		ros::Duration(1.0/RATE).sleep();
	}
	
	// Run example on each camera
	pCam = camList.GetByIndex(0);
	cout << endl << "Running example for camera 0" << "..." << endl;
	// Run example
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	int result = RunSingleCamera(pCam);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Acquired " << result << " images in " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [µs]." << std::endl;
	std::cout << "Rate was: " << result*1E6/ std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " Hz" <<std::endl;
	
	//
	// Release reference to the camera
	//
	// *** NOTES ***
	// Had the CameraPtr object been created within the for-loop, it would not
	// be necessary to manually break the reference because the shared pointer
	// would have automatically cleaned itself up upon exiting the loop.
	//
	pCam = nullptr;
	
	// Clear camera list before releasing system
	camList.Clear();
	
	// Release system
	system->ReleaseInstance();
	//ros::spinOnce();
	//ros::Duration(1.0/RATE).sleep();
	//ros::shutdown();
	return 0;
}
