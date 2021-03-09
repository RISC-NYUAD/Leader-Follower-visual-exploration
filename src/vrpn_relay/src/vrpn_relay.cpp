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

#define RATE 10 //Hz

int ch;

bool first_val=true;
tf::Transform tf_initial_pose, tf_current_pose;
geometry_msgs::PoseStamped pose_out;

ros::Publisher to_mavros_visiontopic, pose_output_topic;

//Define the name for each vicon object using the exact same notation as in vicon server. Dont' mess up!!!
//std::string object_names[num_objects]={"KinovaBase","KinovaGripper", "ZedCamera"};
//std::string object_names[num_objects]={"KinovaBase","KinovaGripper", "MavicProo"};
std::string object_name ("Gapter1");

//OLD FUNCTION_FOR_REFERENCE_ONLY
void vrpnPoseFeedback(geometry_msgs::PoseStampedConstPtr vrpnPose_){
    pose_out=*vrpnPose_;
    //Check for initial pose
    if(first_val){
        tf_initial_pose= tf::Transform(tf::Quaternion(pose_out.pose.orientation.x,
                            pose_out.pose.orientation.y,
                            pose_out.pose.orientation.z,
                            pose_out.pose.orientation.w),
                 tf::Vector3(pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z));
        first_val=false;
    }
    else{
        tf_current_pose= tf::Transform(tf::Quaternion(pose_out.pose.orientation.x,
                    pose_out.pose.orientation.y,
                    pose_out.pose.orientation.z,
                    pose_out.pose.orientation.w),
            tf::Vector3(pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z));
        tf_current_pose=tf_initial_pose.inverseTimes(tf_current_pose);
        pose_out.pose.position.x=tf_current_pose.getOrigin().getX();
        pose_out.pose.position.y=tf_current_pose.getOrigin().getY();
        pose_out.pose.position.z=tf_current_pose.getOrigin().getZ();
        pose_out.pose.orientation.w=tf_current_pose.getRotation().getW();
        pose_out.pose.orientation.x=tf_current_pose.getRotation().getX();
        pose_out.pose.orientation.y=tf_current_pose.getRotation().getY();
        pose_out.pose.orientation.z=tf_current_pose.getRotation().getZ();
        to_mavros_visiontopic.publish(pose_out);
    }
}

void vrpn2localframe(geometry_msgs::PoseStampedConstPtr input_){
    tf::Transform tf_input;
    poseMsgToTF(input_->pose, tf_input);
    tf_input=tf_initial_pose.inverseTimes(tf_input);
    geometry_msgs::Pose output;
    poseTFToMsg(tf_input,output);
    pose_output_topic.publish(output);
}

void vrpnPoseFeedback2(geometry_msgs::PoseStampedConstPtr vrpnPose_){
    pose_out=*vrpnPose_;
    //Check for initial pose
    if(first_val){
        poseMsgToTF(vrpnPose_->pose, tf_initial_pose);
        first_val=false;
    }
    else{
        poseMsgToTF(vrpnPose_->pose, tf_current_pose);
        tf_current_pose=tf_initial_pose.inverseTimes(tf_current_pose);
        poseTFToMsg(tf_current_pose,pose_out.pose);
        pose_out.header=vrpnPose_->header;
        to_mavros_visiontopic.publish(pose_out);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vicon_log");
	ros::NodeHandle nh("~");
	ros::Subscriber vrpn_measurements = nh.subscribe("/vrpn_client_node/"+ object_name + "/pose", 1, &vrpnPoseFeedback2);
	to_mavros_visiontopic = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
    
    ros::Subscriber vrpn_localframe = nh.subscribe("/input_pose", 1, &vrpn2localframe);
    pose_output_topic = nh.advertise<geometry_msgs::PoseStamped>("/output_pose",1);
    
    ros::Rate loop_rate(RATE);
	//MAIN Loop
	while(kbhit()==0 && ros::ok())
	{
		//Spin until user stops program
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
