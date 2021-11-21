#include <iostream>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <ros/duration.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <tf/LinearMath/Quaternion.h>
#include "NE_utilities.h"

using namespace std;

#define takeoff_alt 2.0

std::vector<geometry_msgs::PoseStamped> prev_local_poses, prev_rhombis;
std::vector<geometry_msgs::PoseStamped> upcoming_pose_buffer;
uint32_t pose_buffer;
double delay_time=0.0;

//Set global variables
mavros_msgs::State current_state;
mavros_msgs::RCIn RC_input;
geometry_msgs::PoseStamped Rhombi_input, initial_pose, current_pose, target_pose; //CameraInput, PoseOnGround, Current, Target respectively
double initial_heading, current_heading, target_heading, GYM_OFFSET; //rad

bool experiment_runs;
std_msgs::Bool logger_control, grabbing_control;

double follower_safe_dist = 2.0; //less than this you might be in trouble
double pos_tolerance = 0.25; //when position is considered reached and also value when distances to rhombi are green checked
double rhombi_tolerance = 0.1;
double angle_tolerance = 15; //angular tolerance in degress!!! where no correction from rhombi needed

uint32_t sequence = 0;

//get orientation of the drone from
void getHeading(geometry_msgs::Pose pose_)
{
	tf::Pose posed;
	tf::poseMsgToTF(pose_, posed);
	current_heading = tf::getYaw(posed.getRotation());
}

//set orientation of the drone (drone should always be level) - input is rad
void setHeading(double heading)
{
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(heading+GYM_OFFSET), target_pose.pose.orientation);
}

// set position to fly to in the gym frame
void setDestination(geometry_msgs::Pose dist_)
{
	target_pose.pose.position.x = current_pose.pose.position.x + dist_.position.x - follower_safe_dist;
	target_pose.pose.position.y = current_pose.pose.position.y + dist_.position.y;
	target_pose.pose.position.z = current_pose.pose.position.z + dist_.position.z;
	/* ROS_INFO("Sequence: %d, From X, Y, Z : %lf, %lf, %lf, to X_T, Y_T, Z_T : %lf, %lf, %lf", sequence, current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z, 
	target_pose.pose.position.x,target_pose.pose.position.y, target_pose.pose.position.z); */
}

// set position to fly to in the gym frame
void setDestination2(geometry_msgs::Pose _current_pose, geometry_msgs::Pose _dist )
{
	target_pose.pose.position.x = _current_pose.position.x + _dist.position.x - follower_safe_dist;
	target_pose.pose.position.y = _current_pose.position.y + _dist.position.y;
	target_pose.pose.position.z = _current_pose.position.z + _dist.position.z;
	/* ROS_INFO("Sequence: %d, From X, Y, Z : %lf, %lf, %lf, to X_T, Y_T, Z_T : %lf, %lf, %lf", sequence, current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z, 
	target_pose.pose.position.x,target_pose.pose.position.y, target_pose.pose.position.z); */
}

//stop execution
void start_stop_cb(const std_msgs::Bool input)
{
	ROS_INFO("Start stop data received");
	experiment_runs = input.data;
}

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

geometry_msgs::PoseStamped geometry_msgs_lerp( geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, double ref_time){
	double slope = ( ref_time - pose1.header.stamp.toSec()) / (pose2.header.stamp.toSec() - pose1.header.stamp.toSec());
	geometry_msgs::PoseStamped output;
	output.pose.position.x = NElerp(pose1.pose.position.x, pose2.pose.position.x, slope);
	output.pose.position.y = NElerp(pose1.pose.position.y, pose2.pose.position.y, slope);
	output.pose.position.z = NElerp(pose1.pose.position.z, pose2.pose.position.z, slope);
	tf::Quaternion quat1, quat2;
	tf::quaternionMsgToTF(pose1.pose.orientation, quat1);
    double roll1, pitch1, yaw1;
    tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);
	double roll2, pitch2, yaw2;
	tf::quaternionMsgToTF(pose2.pose.orientation, quat2);
    tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
	tf::Quaternion final;
	final.setRPY(NElerp(roll1, roll2, slope), NElerp(pitch1, pitch2, slope), NElerp(yaw1, yaw2, slope));
	tf::quaternionTFToMsg(final , output.pose.orientation);
	return output;
}

bool pose_lookup(std_msgs::Header msg_header, std::vector<geometry_msgs::PoseStamped> input, geometry_msgs::PoseStamped* output)
{
	//the time we are looking to match is in arg 1, the look up list to look into is arg 2
	int i = 0;
	double ref_time = msg_header.stamp.toSec();
	//ROS_INFO("Seq number: %u", msg_header.seq);
	while(i < input.size() && input.at(i).header.stamp.toSec() < ref_time ){ 
		i++;
	}
	//ROS_INFO("i: %d, input size: %ld", i, input.size());
	//ROS_INFO("Rhombi ref time: %lf. Local_next_pose_time: %lf", msg_header.stamp.toSec(), input.at(i).header.stamp.toSec());
	//ROS_INFO("ROS_TIME_NOW: %lf",ros::Time::now().toSec());
	//Interpolate 
	if(i<2){
		ROS_INFO("No compatible lerp match can be found. i < 2");
		return false;
		//*output = input.at(input.size()-1);
	}
	else
	{
		*output = geometry_msgs_lerp(input.at(i-1) , input.at(i), ref_time);
		output->header = msg_header;
		return true;
		//ROS_INFO("Compatible lerp match found");
		//std::cout << *output << std::endl;
		//ROS_INFO("Before: ");
		//std::cout << input.at(i-1) << std::endl;
		//ROS_INFO("After: ");
		//std::cout << input.at(i) << std::endl;
		
	}
	return false;
}

//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_pose = *msg;
	if ( prev_local_poses.size() >= pose_buffer) prev_local_poses.erase(prev_local_poses.begin());
	prev_local_poses.push_back(current_pose);
	getHeading(current_pose.pose);
}

void rhombi_detect_cb(geometry_msgs::PoseArrayConstPtr input_poses)
{
	//Rotation from Vicon follower to Camera follower: [0 0 1 ; -1 0 0 ; 0 -1 0] converted to quaternion
	tf::Transform V_to_A(tf::Quaternion(-0.5, 0.5, -0.5, 0.5), tf::Vector3(0,0,0));
	//From Camera follower to Marker leader
	tf::Transform A_f_l(tf::Quaternion(0,0,0,1), tf::Vector3(input_poses->poses[0].position.x, input_poses->poses[0].position.y, input_poses->poses[0].position.z));
	//From Vicon follower to Marker leader
	tf::Transform V_to_l = V_to_A * A_f_l;
	tf::poseTFToMsg(V_to_l,Rhombi_input.pose);
	Rhombi_input.header = input_poses->header;
	if (prev_rhombis.size() >= pose_buffer) {
		//prev_rhombis.erase(prev_rhombis.begin() , prev_rhombis.begin() + (prev_rhombis.size() - pose_buffer));
		prev_rhombis.erase(prev_rhombis.begin());
		
	}
	prev_rhombis.push_back(Rhombi_input);
}

void rc_feedback(mavros_msgs::RCInConstPtr RC_input_){
	RC_input = *RC_input_;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "leader_follower_v3_gapter1");
	ros::NodeHandle nh("~");
	//Initialization - Rate
	int RATE = 10;
	nh.getParam("trajectory_rate", RATE);
	ros::Rate loop_rate(RATE);
	
	//Initialization - Object name
	std::string object_name = "~";
	nh.getParam("object_name", object_name);
	
	//Initilization get safe distance
	nh.getParam("follower_distance", follower_safe_dist);
	
	//Initilization get delay time
	nh.getParam("delay_time", delay_time);
	
	double pose_time_buffer = 0;
	nh.getParam("pose_buffer", pose_time_buffer);
	pose_buffer = std::floor(pose_time_buffer * RATE);
	
	// Initialization - Publishers/Subscribers
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
	ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, pose_cb);
	ros::Subscriber experiment_control = nh.subscribe<std_msgs::Bool>("/experiment_control", 1, start_stop_cb);
	ros::Publisher logger_control_ = nh.advertise<std_msgs::Bool>("/logger_control",1);
	ros::Publisher grabbing_control_ = nh.advertise<std_msgs::Bool>("/grabbing_control",1);
	ros::Subscriber rc_input_ = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",1, rc_feedback); 
	
	ROS_INFO("INITIALIZING...");
	for(int i=0; i<10; i++)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	
	// Initialization - Check FCU connection
	while(ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	// Pre-flight - Get the initial orientation of the drone
	GYM_OFFSET = 0;
	int i=0;
	while (ros::ok() && i<30) {
		//for (int i = 1; i <= 30; ++i) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
		GYM_OFFSET += current_heading;
		i++;
	}
	GYM_OFFSET /= i;
	ROS_INFO("the N' axis is facing: %f degrees", rad2deg(GYM_OFFSET));
	
	if(RC_input.channels[8] > 1000){
		ROS_WARN("SET RC9 to low to continue");
		while(ros::ok() &&  RC_input.channels[8] > 1000){
			ros::spinOnce();
			ros::Duration(0.1).sleep();
		}
	}
	
	 std::cout << std::fixed << std::setprecision(10) <<std::endl;
	
	// Pre-flight - Check for and enable guided mode
	mavros_msgs::SetMode guided_set_mode;
	guided_set_mode.request.custom_mode = "GUIDED";
	while(current_state.mode != "GUIDED")
	{
		if( set_mode_client.call(guided_set_mode) && guided_set_mode.response.mode_sent){ ROS_INFO("GUIDED requested");}
		else{ROS_ERROR("Failed to set GUIDED mode");}
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}

	// Pre-flight - Arming
	ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	mavros_msgs::CommandBool srv_arm_i;
	srv_arm_i.request.value = true;
	if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success){
		ROS_INFO("ARM sent %d", srv_arm_i.response.success);
		for(int i=0; i<200; i++)
		{
			ros::spinOnce();
			ros::Duration(0.01).sleep();
		}
	}
	else
	{
		ROS_ERROR("Failed arming");
		return -1;
	}
	initial_pose = current_pose;
	initial_heading = current_heading;
	
 	// Pre-flight - Request takeoff
 	ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
 	mavros_msgs::CommandTOL srv_takeoff;
 	srv_takeoff.request.altitude = takeoff_alt;
 	int stop=0;
 	while (!srv_takeoff.response.success && stop<5){
 		ros::spinOnce();
 		loop_rate.sleep();
 		if(takeoff_cl.call(srv_takeoff)){
 			ROS_INFO("Take off service called successfully");
 			ros::spinOnce();
 			loop_rate.sleep();
 			if(srv_takeoff.response.success){
 				ROS_INFO("takeoff success");
 				stop=10;
 			}
 			else{	
 				ROS_INFO("takeoff fail. Send success: %d", srv_takeoff.response.success);
 				stop++;
 			}
 		}
 		else{
 			ROS_INFO("Could not make the takeoff service call"); stop++;
 		}
 		ros::spinOnce();
 		ros::Duration(1).sleep();
 	}

 	// Flight - Waiting to reach takeoff alt
 	while(current_pose.pose.position.z < (initial_pose.pose.position.z + takeoff_alt-0.1))
 	{
 		ros::spinOnce();
 		loop_rate.sleep();
 	}
	
	// Waiting to start experiment
	experiment_runs = false;
	ROS_WARN("Flip the RC switch to start experiment!!!!");
	while(ros::ok() && !experiment_runs){
		if(RC_input.channels[8] > 1000) experiment_runs = true;
		ros::spinOnce();
		loop_rate.sleep();
	}
	target_pose = current_pose;
	target_heading = current_heading;
	
	//Starting all other executables
	ROS_INFO("Publish start all");
	logger_control.data = true;
	grabbing_control.data = true;
	grabbing_control_.publish(grabbing_control);
	logger_control_.publish(logger_control);
	ros::spinOnce();
	loop_rate.sleep();
	ROS_INFO("Done start all");
	
	//Subscribe to start getting reference
	ros::Subscriber rhombi_pose_ = nh.subscribe<geometry_msgs::PoseArray>("/"+ object_name +"/pose_stamped",1, rhombi_detect_cb);
	ros::spinOnce();
	loop_rate.sleep();
	sequence = Rhombi_input.header.seq;
	
	ROS_INFO("Starting following");
	// Flight - Moving between WP
	geometry_msgs::PoseStamped applied_pose;
	applied_pose = target_pose;
	while(ros::ok() && experiment_runs){
		//if you saw a new rhombi
		if(Rhombi_input.header.seq!=sequence){
			if(abs(Rhombi_input.pose.position.x-follower_safe_dist)>rhombi_tolerance || abs(Rhombi_input.pose.position.y)>rhombi_tolerance ||
			abs(Rhombi_input.pose.position.z)>rhombi_tolerance){
				if(prev_rhombis.size() > 1 && prev_local_poses.size() > 1){
					geometry_msgs::PoseStamped output, output2;
					if (pose_lookup(Rhombi_input.header, prev_rhombis, &output) && pose_lookup(Rhombi_input.header, prev_local_poses, &output2)){	
						setDestination2(output.pose, output2.pose);
						setHeading(target_heading);
						target_pose.header = Rhombi_input.header;
						if ( upcoming_pose_buffer.size() >= pose_buffer) upcoming_pose_buffer.erase(upcoming_pose_buffer.begin());
						upcoming_pose_buffer.push_back(target_pose);
						sequence = Rhombi_input.header.seq;
					}
					else ROS_INFO("Loop out");
				}
			}
		}
		//Impose time delay
		if(upcoming_pose_buffer.size() > 0){
			ros::Time ts = ros::Time::now();
			int s = 0;
			//Move until you find the first measurement that is satisfying the delay time
			while(s < upcoming_pose_buffer.size() && ts.toSec() - upcoming_pose_buffer.at(s).header.stamp.toSec() < delay_time){
				s++;
			}
			while(s<upcoming_pose_buffer.size()){
				//If next measurement is also satisfying the delay time
				if(s<upcoming_pose_buffer.size()-1 && ts.toSec() - upcoming_pose_buffer.at(s+1).header.stamp.toSec() >= delay_time){
					s++;
				}
				//Else impose this one
				else{
					//ROS_INFO()
					//double dif = ts.toSec() - upcoming_pose_buffer.at(s).header.stamp.toSec();
					//std::cout << "Dif: " << dif << " delay: " << delay_time << std::endl << std::endl;
					applied_pose.header = upcoming_pose_buffer.at(s).header;
					applied_pose.pose.position = upcoming_pose_buffer.at(s).pose.position;
					ROS_INFO("A.Upcoming pose buffer size: %ld and s is %d", upcoming_pose_buffer.size(),s);
					if(s>0) upcoming_pose_buffer.erase(upcoming_pose_buffer.begin(), upcoming_pose_buffer.begin()+s);
					else upcoming_pose_buffer.erase(upcoming_pose_buffer.begin());
					ROS_INFO("B.Upcoming pose buffer size: %ld and s is %d", upcoming_pose_buffer.size(),s);
					//std::cout << "Time diff: " << ts.toSec()- applied_pose.header.stamp.toSec()<< "Delay: " << delay_time << std::endl;
					break;
				}
			}
		}
		//std::cout << "Applied pose: " << applied_pose << std::endl;
		local_pos_pub.publish(applied_pose);
		ros::spinOnce();
		loop_rate.sleep();
		if(RC_input.channels[8] < 1000) experiment_runs = false;
	}
	
	//Stoping all other executables
	ROS_INFO("Publish end all");
	logger_control.data = false;
	grabbing_control.data = false;
	grabbing_control_.publish(grabbing_control);
	logger_control_.publish(logger_control);
	ros::spinOnce();
	loop_rate.sleep();
	ROS_INFO("Done end all");
	
	// Finish - Land
	ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	mavros_msgs::CommandTOL srv_land;
	if (land_client.call(srv_land) && srv_land.response.success) ROS_INFO("Land commensing %d", srv_land.response.success);
	else ROS_ERROR("Landing failed");

	while((current_pose.pose.position.z - initial_pose.pose.position.z)>pos_tolerance)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	return 0;
}
