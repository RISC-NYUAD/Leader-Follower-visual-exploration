/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * 	Copyright (C) 2018, Nikolaos Evangeliou, NYU Abu Dhabi, Robotics Lab
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
#include <tf/tf.h>

tf::Transform tf_initial_pose;
geometry_msgs::PoseStamped pose_out;

void vrpnPoseFeedback(geometry_msgs::PoseStampedConstPtr vrpnPose_){
    pose_out=*vrpnPose_;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vrpn_relay");
	ros::NodeHandle nh("~");
	
	//Initialization - Rate
	int RATE = 10;
	nh.getParam("vrpn_input_rate", RATE);
	ros::Rate loop_rate(RATE);
	
	int OUTPUT_RATE = 10; 
	nh.getParam("vrpn_output_rate", OUTPUT_RATE);
	double delay_time = (1.0/OUTPUT_RATE);
	
	//Initialization - Object name
	std::string object_name = "~";
	nh.getParam("object_name", object_name);
	
	std::cout << std::fixed << std::setprecision(10);

	ros::Subscriber vrpn_measurements = nh.subscribe("/vrpn_client_node/"+ object_name + "/pose", 1, &vrpnPoseFeedback);
	ros::Publisher to_mavros_visiontopic = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);

	while(pose_out.header.seq<=0){
		//Spin until first frame received
		ros::spinOnce();
		loop_rate.sleep();
	}
	poseMsgToTF(pose_out.pose, tf_initial_pose);
	int frameID = pose_out.header.seq;
	double previous_publish_time_sec = ros::Time::now().toSec();

	//MAIN Loop
	while(ros::ok()){
		//Spin and publish until user stops program
		if(frameID != pose_out.header.seq  && pose_out.header.stamp.toSec() - previous_publish_time_sec >= delay_time){
			frameID = pose_out.header.seq;
			previous_publish_time_sec = pose_out.header.stamp.toSec();
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
