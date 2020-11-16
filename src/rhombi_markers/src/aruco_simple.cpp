/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
 

#include <iostream>
#include "aruco.h"
#include <cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat srcImg, dstImg;
aruco::MarkerDetector MDetector;
std::vector<aruco::Marker> Markers;

std::string image_encoding="mono8"; //options in cv are: "mono8" "bgr8" "bgra8" "rgb8" "rgba8" "mono16"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
	//ARUCO START	
	srcImg = cv_bridge::toCvShare(msg, image_encoding)->image;
        dstImg = srcImg;
		
        //for each marker, draw info and its boundaries in the image
        for(auto m:MDetector.detect(srcImg)){
            std::cout<< m << std::endl;
            m.draw(dstImg);            
        }
        cv::imshow("out",dstImg);
        cv::waitKey(1);//wait for key to be pressed
		
		
	}
	catch (cv_bridge::Exception& e){
	  ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
}

int main(int argc,char **argv)
{   
	ros::init(argc, argv, "aruco_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    //image_transport::Subscriber sub = it.subscribe("/flir_adk/image_raw", 1, imageCallback);
	image_transport::Subscriber sub = it.subscribe("/camera_array/cam0/image_raw", 1, imageCallback);
    	MDetector.setDictionary("ARUCO_MIP_36h12");
    //MDetector.setDetectionMode(aruco::DetectionMode::DM_FAST,0.02);
    	ros::spin();
}


