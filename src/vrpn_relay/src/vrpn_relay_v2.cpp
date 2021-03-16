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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "NE_utilities.h"

tf::Transform tf_initial_pose;
geometry_msgs::PoseStamped pose_out;

void vrpnPoseFeedback2(geometry_msgs::PoseStampedConstPtr vrpnPose_){
    pose_out=*vrpnPose_;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vrpn_relay");
	ros::NodeHandle nh("~");
	
	//Initialization - Rate
	int RATE = 10;
	nh.getParam("logging_rate", RATE);
	ros::Rate loop_rate(RATE);
	
	//Initialization - Object name
	std::string object_name = "~";
	nh.getParam("object_name", object_name);
	
	ros::Subscriber vrpn_measurements = nh.subscribe("/vrpn_client_node/"+ object_name + "/pose", 1, &vrpnPoseFeedback2);
	ros::Publisher to_mavros_visiontopic = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
	
	while(!(pose_out.header.seq>0)){
		//Spin until first frame received
		ros::spinOnce();
		loop_rate.sleep();
	}
	poseMsgToTF(pose_out.pose, tf_initial_pose);
	int frameID = pose_out.header.seq;
	
	//MAIN Loop
	while(kbhit()==0 && ros::ok())
	{
		//Spin and publish until user stops program
		if(frameID != pose_out.header.seq){
			frameID = pose_out.header.seq;
			tf::Transform tf_current_pose;
			poseMsgToTF(pose_out.pose, tf_current_pose);
			tf_current_pose=tf_initial_pose.inverseTimes(tf_current_pose);
			poseTFToMsg(tf_current_pose,pose_out.pose);
			to_mavros_visiontopic.publish(pose_out);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
