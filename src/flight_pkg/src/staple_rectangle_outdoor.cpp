#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <tf/LinearMath/Quaternion.h>
#include "NE_utilities.h"

using namespace std;

#define takeoff_alt 5.0
#define rect_size 3.0
#define rect_height 1.0
#define NUM_POINTS 10
#define TOLERANCE 1.0 //m
#define TIME_TOLERANCE 30 //seconds
#define RATE 20 //Hz

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped initial_pose, current_pose, target_pose;
float current_heading,GYM_OFFSET;
bool stop_exec;


// float points[NUM_POINTS][4]={{0.0, rect_size, takeoff_alt, -90},
// 						{0.0, rect_size, takeoff_alt/2.0, 0},
// 							{rect_size, rect_size, takeoff_alt/2.0, 0},
// 								{rect_size, rect_size, takeoff_alt, 90},
// 									{rect_size, 0.0, takeoff_alt, 90},
// 										{rect_size, 0.0, takeoff_alt/2.0, -180},
// 											{0.0, 0.0, takeoff_alt/2.0, -180},
// 												{0.0, 0.0, takeoff_alt, 0}};


//X right, Y forward for whatever reason when outdoor
//rect_size 3 rect_height 2
float points1[NUM_POINTS][4]={{0.0, 0.0, rect_height, 0},
                                {rect_size, 0.0, rect_height, 0},
                                    {rect_size, 0.0, -rect_height, 0},
                                        {rect_size, -rect_size, -rect_height, 0},
                                            {rect_size, -rect_size, rect_height, 0},
                                                {-rect_size, -rect_size, rect_height, 0},
                                                    {-rect_size, -rect_size, -rect_height, 0},
                                                        {-rect_size, 0.0, -rect_height, 0},
                                                            {-rect_size, 0.0, rect_height, 0},
                                                                    {0.0, 0.0, rect_height, 0}};
                                                
float points[NUM_POINTS][4]={{0.0, 0.0, rect_size/2, 0},
						{0.0, rect_size/2, rect_size/2, -90},
							{0.0, rect_size/2, -rect_size/2, 0},
								{rect_size, rect_size/2, -rect_size/2, 0},
									{rect_size, rect_size/2, rect_size/2, 90},
										{rect_size, -rect_size/2, rect_size/2, 90},
                                        {rect_size, -rect_size/2, -rect_size/2, 180},
                                        {0.0, -rect_size/2, -rect_size/2, 180},
											{0.0, -rect_size/2, rect_size/2, 0},
												{0.0, 0.0, rect_size/2, 0}};                            

//set orientation of the drone (drone should always be level)
void setHeading(float heading)
{
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(heading), target_pose.pose.orientation);
}

// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
  target_pose.pose.position.x = x*cos(-GYM_OFFSET) - y*sin(-GYM_OFFSET);
  target_pose.pose.position.y = x*sin(-GYM_OFFSET) + y*cos(-GYM_OFFSET);
  target_pose.pose.position.z = z;
  ROS_INFO("Destination set to x: %f y: %f z %f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
}

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
}

//get current compass heading of drone
void compass_cb(const std_msgs::Float64ConstPtr& compass_hdg_)
{
  current_heading=deg2rad(compass_hdg_->data);
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
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Rate rate(20.0); // the setpoint publishing rate MUST be faster than 2Hz
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
  ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 2);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 2);
  ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, pose_cb);
  ros::Subscriber stop_execution = nh.subscribe<std_msgs::Bool>("/stop_experiment", 1, stop_cb);
  ros::Subscriber compass_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 1, compass_cb);

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
  
  //set the orientation of the gym
  ROS_INFO("Fixing heading");
  GYM_OFFSET = 0;
  for (int i = 0; i <30; i++) {
    ros::spinOnce();
    rate.sleep();
    GYM_OFFSET += current_heading;
  }
  GYM_OFFSET /= 30;
  ROS_INFO("the N' axis is facing: %f", rad2deg(GYM_OFFSET));
  
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

  //Make sure we have received our pose at least Once
  ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/mavros/local_position/pose", nh);
  initial_pose = current_pose;
  
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
  for(int i=0; i<200; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  
  //request takeoff
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = initial_pose.pose.position.z+takeoff_alt;
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

  // Hover for 10 seconds
  ROS_INFO("HOVERING...");
  for(int i=0; i<1000; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  //Do setpoint
  geometry_msgs::Pose rectangle_center = initial_pose.pose; 
  rectangle_center.position.z +=takeoff_alt;  //Takeoff pose is the center of the rectangle
  double wp_elapsed;
  for(uint8_t j=0;j<NUM_POINTS;j++){
    ROS_INFO("Waypoint %d",j);
    wp_elapsed = ros::Time::now().toSec();
    setHeading(GYM_OFFSET+deg2rad(points1[j][3]));
    setDestination(rectangle_center.position.x+points1[j][0], 
                        rectangle_center.position.y+points1[j][1], 
                            rectangle_center.position.z+points1[j][2]);
    while((ros::Time::now().toSec() - wp_elapsed) < TIME_TOLERANCE && ros::ok())
    {
        local_pos_pub.publish(target_pose);
        if( get_distance3D(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                            current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z) < TOLERANCE) {
            ROS_INFO("Finished waypoint");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
  }
  
  //land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }
  
  ROS_INFO("FINISHING...");
  for(int i=0; i<1000; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

//   while (ros::ok())
//   {
//     ros::spinOnce();
//     rate.sleep();
//   }
  return 0;
}
