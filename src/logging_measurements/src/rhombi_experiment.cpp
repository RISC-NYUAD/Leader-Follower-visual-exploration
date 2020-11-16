/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  Copyright (c) 2011, Markus Achtelik, ETH Zurich, Autonomous Systems Lab (modifications)
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
 *   * Neither the name of the University of California nor the names of its
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
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include "NE_utilities.h"

#define RATE 10 //Hz

std::string object_name="Gapter_2";
std::ofstream outfile;
ros::Time exp_init;
double elapsed = 0.0;

geometry_msgs::TwistStamped FCUvel;
geometry_msgs::PoseStamped  FCUPose;
geometry_msgs::PoseArray RhombiPoses;

bool has_run1,has_run2,has_run3;
bool experiment_runs=true;

void vrpnPoseFeedback(geometry_msgs::PoseStampedConstPtr vrpnPose_){
    //elapsed = ros::Time::now().toSec()-exp_init.toSec();
    //Boost time returns the time based on computer clock
    //boost::posix_time::ptime date_time = boost::posix_time::microsec_clock::universal_time();
    //elapsed = seconds_from_epoch(date_time);
    if(has_run1 && has_run2 && has_run3){
    outfile << ros::Time::now() << ";"
	//VRPN pose
        << vrpnPose_->pose.position.x << ";" << vrpnPose_->pose.position.y << ";" << vrpnPose_->pose.position.z << ";" 
        << vrpnPose_->pose.orientation.w << ";" <<  vrpnPose_->pose.orientation.x << ";" <<  vrpnPose_->pose.orientation.y << ";" <<  vrpnPose_->pose.orientation.z << ";" 
	<< FCUvel.twist.linear.x << ";" << FCUvel.twist.linear.y << ";" << FCUvel.twist.linear.z << ";"
        //Flgiht controller pose
	<< FCUPose.pose.position.x << ";" << FCUPose.pose.position.y << ";" << FCUPose.pose.position.z << ";"
        << FCUPose.pose.orientation.w << ";" << FCUPose.pose.orientation.x << ";" << FCUPose.pose.orientation.y << ";" << FCUPose.pose.orientation.z << ";"
        //Discovered rhombis pose
        <<RhombiPoses.header.seq << ";" << RhombiPoses.header.stamp << ";" ;
        for(auto n : RhombiPoses.poses)
        outfile << n.position.x << ";" << n.position.y << ";" << n.position.z << ";" << n.orientation.x << ";" << n.orientation.y << ";" << n.orientation.z << ";" << n.orientation.w << ";"
        << std::endl;
    }
}

void RhombiPose_cb(geometry_msgs::PoseArrayConstPtr arrayPose_){
	has_run1 = true;
	RhombiPoses = *arrayPose_;
}

void FCUvelocity_cb(geometry_msgs::TwistStampedConstPtr FCUVel_){
  	has_run2 = true;
	FCUvel = *FCUVel_;
}

void FCUPose_cb(geometry_msgs::PoseStampedConstPtr FCUPose_){
  	has_run3 = true;
	FCUPose = *FCUPose_;
}

void experimentHandleFeedback(std_msgs::Bool var){
	if(experiment_runs && var.data==false) experiment_runs=false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rhombi_log_"+object_name);
	ros::NodeHandle nh("~");
	ros::Subscriber vrpn_measurements = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ object_name + "/pose", 1, &vrpnPoseFeedback);
	ros::Subscriber currentVel = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &FCUvelocity_cb);
	ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &FCUPose_cb);
	ros::Subscriber rhombiPos= nh.subscribe<geometry_msgs::PoseArray>("/"+object_name+"/pose_stamped", 1, &RhombiPose_cb);
	ros::Rate loop_rate(RATE);
    	ros::Subscriber tracking_Start=nh.subscribe<std_msgs::Bool> ("/rhombi_det_execution",1, &experimentHandleFeedback);
	//   Initialization - Log file
	std::string usr=get_username();
	std::string fileName;
	//try to get username, if not save to home folder
	if(usr!="?") fileName="/home/"+usr+"/projects/nevangeliou_GapterUAV/logs/Rhombi_Experiments/" + object_name + "log_"+date_filename();
	else fileName=date_filename();
	outfile.open(fileName, std::ios::out | std::ios::app);
	if (outfile.fail()){
	    throw std::ios_base::failure(std::strerror(errno));
	    return -1;
	  }
	ROS_INFO("Saving data to: %s", fileName.c_str());
	// 	make sure write fails with exception if something is wrong
	outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit);
	outfile << "time_elapsed(Sec)"<< ";" 
	<< "Vicon position (x)" << ";" << "y" << ";" << " z" << ";" 
	<< "Vicon orientation (q_w)" << ";" <<  "q_x" << ";" <<  "q_y" << ";" <<  "q_z" << ";"
	<< "FCU linear velocity (v_x)" << ";" <<  "v_y" << ";" <<  "v_z" << ";"
	<< "FCU local position (x)" << ";" <<  "y" << ";" <<  "z" << ";"
	<< "FCU local orientation (q_w)" << ";" <<  "q_x" << ";" <<  "q_y" << ";" <<  "q_z" << ";" 
	<< "Rhombi frame detection seq" << ";" << "Rhombi frame detection time stamp" << ";"
	<< "Rhombis detected position" << ";" <<  "x" << ";" <<  "y" << ";" <<  "z" << ";" 
	<< "Rhombis detected orientation (q)" << ";" <<  "q_x" << ";" <<  "q_y" << ";" <<  "q_z" << ";" 
	<<std::endl;
	//Dummy init to avoid writing before all callbacks run
	has_run1=has_run2=has_run3=false;
	//MAIN Loop
	while(kbhit()==0 && ros::ok() && experiment_runs)
	    {
		    //Spin until user stops program
		    ros::spinOnce();
		    loop_rate.sleep();
	    }
	ROS_INFO("Saved data to: %s", fileName.c_str());
	outfile.close();
	    return 0;
}
