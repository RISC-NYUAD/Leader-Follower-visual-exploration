/*********************************************************************
 * Software License Agreement (BSD License)
 * 	Copyright (C) 2018, Nikolaos Evangeliou, NYU Abu Dhabi, Robotics Lab (modifications)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the NYU Abu Dhabi nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include "NE_utilities.h"

std::ofstream outfile;

geometry_msgs::PoseStamped  CCDPose;
bool experiment_runs=false;
const int seconds_in_hour = 3600;

int frameID = -1;
int offset_hours = 8; //default

void CCDPoseFeedback(geometry_msgs::PoseStampedConstPtr CCDPose_){
		CCDPose = *CCDPose_;
		outfile << CCDPose.header.stamp.toSec() -  offset_hours*seconds_in_hour << ";"
		<< CCDPose.pose.position.x << ";" << CCDPose.pose.position.y << ";" << CCDPose.pose.position.z << ";" 
		<< CCDPose.pose.orientation.w << ";" << CCDPose.pose.orientation.x<<";"<< CCDPose.pose.orientation.y<< ";"<<CCDPose.pose.orientation.z << std::endl;
}

void loggerHandleFeedback(std_msgs::Bool var){
	experiment_runs= var.data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "matrice_logger");
	ros::NodeHandle nh("~");
	//Initialization - Rate
	int RATE = 100;
	nh.getParam("logger_logging_rate", RATE);
	ros::Rate loop_rate(RATE);
	
	//Initialization - Object name
	std::string object_name = "~";
	nh.getParam("object_name", object_name);
	
	nh.getParam("vicon_server_offset", offset_hours);
	
	//Initialization - Log file
	std::string fileName = "~";
	nh.getParam("logging_path", fileName);
	if(!fileName.empty()) fileName += object_name + "_" + date_filename();
	else fileName = object_name + "_"+ date_filename();
	outfile.open(fileName, std::ios::out | std::ios::app);
	if (outfile.fail()){
		ROS_INFO("Could not create logfile");
		throw std::ios_base::failure(std::strerror(errno));
		return -1;
    }
	outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit); //write fails with exception if something is wrong
	//VRPN Pose
	outfile << "Matrice_timestamp(epoch)"<< ";" 
	<< "Matrice_pos(x)" << ";" << "y" << ";" << " z" << ";" << "Matrice_quat(q_w)" << ";" <<  "q_x" << ";" <<  "q_y" << ";" <<  "q_z" << std::endl;
	
	outfile << std::fixed << std::setprecision(10);
	
	//Initialization - Publishers Subscribers
	ros::Subscriber Matrice_Pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ object_name + "/pose", 1, &CCDPoseFeedback);
	ros::Subscriber tracking_Start_sub =nh.subscribe<std_msgs::Bool> ("/logger_control",1, &loggerHandleFeedback);
	
	//Wait for all callbacks to run -- Hard constraint
	while(Matrice_Pos_sub.getNumPublishers()==0){
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Subscribers connected. Saving data to: %s. Waiting for exp to start.", fileName.c_str());
	while(ros::ok() && !experiment_runs)
    {
		    ros::spinOnce();
		    loop_rate.sleep();
    }
	
	//MAIN Loop
	while(ros::ok() && experiment_runs)
    {
		    ros::spinOnce();
		    loop_rate.sleep();
    }
	ROS_INFO("Saved data to: %s", fileName.c_str());
	outfile.close();
    return 0;
}
