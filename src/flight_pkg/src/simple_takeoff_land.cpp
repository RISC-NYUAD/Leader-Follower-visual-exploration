#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <ros/duration.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <tf/LinearMath/Quaternion.h>
#include "NE_utilities.h"

using namespace std;

#define takeoff_alt 2.0
#define rect_size 1.5

std::string drone_name="Gapter_2";

#define NUM_POINTS 8

#define AIR_TIME 100 //in seconds

#define RATE 10 //Hz

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped initial_pose, current_pose, target_pose;
double current_heading; //rad
bool stop_exec=false;
float GYM_OFFSET;

float tollerance = .10;

std::ofstream outfile;

float points[NUM_POINTS][4]={{0.0, rect_size, takeoff_alt, -90},
						{0.0, rect_size, takeoff_alt/2.0, 0},
							{rect_size, rect_size, takeoff_alt/2.0, 0},
								{rect_size, rect_size, takeoff_alt, 90},
									{rect_size, 0.0, takeoff_alt, 90},
										{rect_size, 0.0, takeoff_alt/2.0, -180},
											{0.0, 0.0, takeoff_alt/2.0, -180},
												{0.0, 0.0, takeoff_alt, 0}};

//get orientation of the drone from
void getHeading(geometry_msgs::Pose pose_)
{
  tf::Quaternion q(
    pose_.orientation.x,
    pose_.orientation.y,
    pose_.orientation.z,
    pose_.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_heading=yaw;
}

//set orientation of the drone (drone should always be level)
void setHeading(float heading)
{
  //heading = -heading + 90 - GYM_OFFSET;
  float yaw = -heading*M_PI/180 - GYM_OFFSET;
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  target_pose.pose.orientation.w = qw;
  target_pose.pose.orientation.x = qx;
  target_pose.pose.orientation.y = qy;
  target_pose.pose.orientation.z = qz;
}

// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
  float X = x*cos(-GYM_OFFSET) - y*sin(-GYM_OFFSET);
  float Y = x*sin(-GYM_OFFSET) + y*cos(-GYM_OFFSET);
  float Z = z;
  target_pose.pose.position.x = X;
  target_pose.pose.position.y = Y;
  target_pose.pose.position.z = Z;
  ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}

//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
  getHeading(current_pose.pose);
}

//stop execution
void stop_cb(const std_msgs::Bool input)
{
  ROS_INFO("Stop data received");
  //stop_exec = input.data;
  stop_exec = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_take_off_land_"+drone_name);
  ros::NodeHandle nh;
  ROS_INFO("Start");
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(RATE);
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
  //ros::Publisher stop_execution_pub = nh.advertise<std_msgs::Bool>("/stop_execution", 1);
 // ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
  ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, pose_cb);
  ros::Publisher start_stop_execution = nh.advertise<std_msgs::Bool>("/rhombi_det_execution", 1);
  ros::Subscriber stop_execution = nh.subscribe<std_msgs::Bool>("/stop_experiment", 1, stop_cb);

  // allow the subscribers to initialize
  ROS_INFO("INITIALIZING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  
  ROS_INFO("CHECKING FCU CONNECTION...");
  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  
  //Check for and enable guided mode
  mavros_msgs::SetMode guided_set_mode;
  guided_set_mode.request.custom_mode = "GUIDED";
  //while(current_state.mode != "GUIDED" && (ros::Time::now() - last_request > ros::Duration(5.0)))
  while(current_state.mode != "GUIDED")
  {
	if( set_mode_client.call(guided_set_mode) && guided_set_mode.response.mode_sent){ ROS_INFO("GUIDED requested");}
	else{ROS_ERROR("Failed to set GUIDED mode");}
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }

  //set the orientation of the gym
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
  ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);

  // arming
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm_i;
  srv_arm_i.request.value = true;
  if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
    ROS_INFO("ARM sent %d", srv_arm_i.response.success);
  else
  {
    ROS_ERROR("Failed arming");
    return -1;
  }
  for(int i=0; i<500; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  //request takeoff
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = takeoff_alt;
  int stop=0;
  while (!srv_takeoff.response.success && stop<5){
	  ros::spinOnce();
	  rate.sleep();
	  if(takeoff_cl.call(srv_takeoff)){
		  ROS_INFO("Take off service called successfully");
		  ros::spinOnce();
		  rate.sleep();
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

  // Hover for AIR_TIME seconds
  ROS_INFO("HOVERING...");
    //Publish to start camera acquisition
  ros::Time begin=ros::Time::now();
  double elapsed=0.0;
  std_msgs::Bool rhombi_detection;
  rhombi_detection.data=true;
    //start_stop_execution.publish(rhombi_detection);
	ros::spinOnce();
	rate.sleep();
  while(elapsed<AIR_TIME && !stop_exec)
  {
	ros::spinOnce();
	rate.sleep();
	elapsed = ros::Time::now().toSec() - begin.toSec();
  }
  
  //Stop Rhombi detection
  rhombi_detection.data=false;
  start_stop_execution.publish(rhombi_detection);
  ros::spinOnce();
  rate.sleep();
  
  //land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  if (land_client.call(srv_land) && srv_land.response.success) ROS_INFO("land sent %d", srv_land.response.success);
  else ROS_ERROR("Landing failed");
  
  ROS_INFO("FINISHING...");
  for(int i=0; i<1000; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  return 0;
}
