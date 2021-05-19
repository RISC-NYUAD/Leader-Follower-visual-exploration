#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/Bool.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <tf/LinearMath/Quaternion.h>
#include <time.h>
#include "NE_utilities.h"

#define RATE 25

#define takeoff_alt 2.5
#define circle_radius 0.75
#define circle_period 8 /*seconds to do a full circle*/                           
#define no_circles 1
#define final_alt 3.5
const double omega = 2*M_PI/circle_period;

// insert here maximum drone velocities
const double UAV_Vx_max = 7.7;
const double UAV_Vy_max = 7.2;
const double UAV_Vz_max = 2.7;

const float p = 0.2f;
const float i = 0.0;//005f;
const float d = 0.0f;

const float tollerance = 0.1f;

ros::Time init_time;
double elapsed_time = 0.0;;

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose initial_pose;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::TwistStamped target_vel;
geometry_msgs::TwistStamped current_vel;


double current_heading; //rad
bool stop_exec=false;
float GYM_OFFSET;

std::ofstream outfile;

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
    float yaw = -heading - GYM_OFFSET;
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
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.seq += 1;
    target_pose.pose.position.x = X;
    target_pose.pose.position.y = Y;
    target_pose.pose.position.z = Z;
    //ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

//stop execution
void stop_cb(const std_msgs::Bool input)
{
    ROS_INFO("Stop data received");
    //stop_exec = input.data;
    stop_exec = true;
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
    current_pose = msg->pose;
    getHeading(current_pose);
}

void FCUvelocity_cb(geometry_msgs::TwistStampedConstPtr FCUVel_){
    current_vel = *FCUVel_;
}

void update_velocities_screw(double elapsed_time_)
{
    //x=R*cos(omega*t),  y=R*sin(omega*t)
    // velocity is expressed with respect to 
    target_vel.twist.linear.z = -omega*circle_radius*sin(omega* elapsed_time_);
    target_vel.twist.linear.y =  omega*circle_radius*cos(omega* elapsed_time_);
    target_vel.twist.linear.x =  (final_alt - takeoff_alt)/(circle_period*no_circles);
    ROS_INFO("Current Ref velocities at %f sec: X: %lf, Y: %lf, Z: %lf ", elapsed_time_,  target_vel.twist.linear.x,  target_vel.twist.linear.y, target_vel.twist.linear.z);
    // Calculate reference position as well for logging purposes
    setHeading(0);
    setDestination(elapsed_time_*(final_alt - takeoff_alt)/(circle_period*no_circles), circle_radius*sin(omega* elapsed_time_), circle_radius*cos(omega* elapsed_time_));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Publisher stop_execution_pub = nh.advertise<std_msgs::Bool>("/stop_execution", 10);
    ros::Publisher set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber currentVel = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &FCUvelocity_cb);
    ros::Publisher set_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber stop_execution = nh.subscribe<std_msgs::Bool>("/stop_execution", 10, stop_cb);

    // allow the subscribers to initialize
    ROS_INFO("INITIALIZING...");
    for(int i=0; i<50; i++)
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

    //   Initialization - Log file
    std::string usr=get_username();
    std::string fileName;
    //try to get username, if not save to home folder
    if(usr!="?") fileName="/home/"+usr+"/projects/gapter_UAV/logs/"+date_filename();
    else fileName=date_filename();
    outfile.open(fileName, std::ios::out | std::ios::app);
    if (outfile.fail()){
        throw std::ios_base::failure(std::strerror(errno));
        return -1;
      }

    // 	make sure write fails with exception if something is wrong
    outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit);
    outfile << "time_elapsed(Sec)"<< ";" 
    << "Gapter Reference Position (X^r)" << ";" << "Y^r" << ";" << "Z^r" << ";" 
    << "Gapter Reference Velocity (v^r_x)" << ";" <<  "v^r_y" << ";" <<  "v^r_z" << ";" 
    << "Gapter Position (X)" << ";" << "Y" << ";" << "Z" << ";" 
    << "Gapter Velocity LocalFrame! (v_x)" << ";" <<  "v_y" << ";" <<  "v_z" << std::endl;

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
    for(int i=0; i<200; i++)
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

    // Hover for 5 seconds and get current pose
    ROS_INFO("if (!check_cage_boundaries()) HOVERING...");
    for(int i=0; i<500; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    
    // Go to home
    ROS_INFO("Returning to home and landing.");
    setHeading(0);
    setDestination(0.0, 0.0, takeoff_alt);
    double dMag = get_distance3D(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,current_pose.position.x,current_pose.position.y,current_pose.position.z);
    while(ros::ok() && dMag>=tollerance){
	ROS_INFO("Return home");
        set_pos_pub.publish(target_pose);
        dMag = get_distance3D(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,current_pose.position.x,current_pose.position.y,current_pose.position.z);
        ros::spinOnce();
        rate.sleep();
    }
    
    // Hover for 5 seconds and get current pose
    ROS_INFO("if (!check_cage_boundaries()) HOVERING...");
    for(int i=0; i<500; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    initial_pose = current_pose;
    
    //Initialize exp values 
    init_time = ros::Time::now();
    target_vel.twist.linear.x = target_vel.twist.linear.y = target_vel.twist.linear.z =target_vel.twist.angular.x = target_vel.twist.angular.y =target_vel.twist.angular.z = 0;
    // MAIN EXP LOOP
    while(ros::ok() && !stop_exec && kbhit()==0 && elapsed_time <= no_circles*circle_period){
        // update and publish velocities
        elapsed_time = ros::Time::now().toSec()-init_time.toSec();
        update_velocities_screw(elapsed_time);
        set_vel_pub.publish(target_vel);
        outfile << elapsed_time << ";" 
        << target_pose.pose.position.x << ";" << target_pose.pose.position.y << ";" << target_pose.pose.position.z << ";"  
        << target_vel.twist.linear.x << ";" << target_vel.twist.linear.y << ";" << target_vel.twist.linear.z << ";" 
        << current_pose.position.x << ";" <<  current_pose.position.y << ";" <<  current_pose.position.z << ";"  
        << current_vel.twist.linear.x << ";" << current_vel.twist.linear.y << ";" << current_vel.twist.linear.z <<std::endl;
        ros::spinOnce();
        rate.sleep();
      }
    ROS_INFO("Timed out.");
    // Zero the velocities
    target_vel.twist.linear.x = 0;
    target_vel.twist.linear.y = 0;
    target_vel.twist.linear.z = 0;
    set_vel_pub.publish(target_vel);
    ros::spinOnce();
    rate.sleep();
    set_vel_pub.publish(target_vel);
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("Saved data to: %s", fileName.c_str());
    outfile.close();
    // Go to home
    ROS_INFO("Returning to home and landing.");
    setHeading(0);
    setDestination(0.0, 0.0, takeoff_alt);
    double dMag2 = get_distance3D(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,current_pose.position.x,current_pose.position.y,current_pose.position.z);
    while(ros::ok() && dMag2>=tollerance){
	ROS_INFO("Return home");
        set_pos_pub.publish(target_pose);
        dMag2 = get_distance3D(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,current_pose.position.x,current_pose.position.y,current_pose.position.z);
        ros::spinOnce();
        rate.sleep();
      }
    //land
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success) ROS_INFO("land sent %d", srv_land.response.success);
    else ROS_ERROR("Landing failed");

    ROS_INFO("FINISHING...");
    for(int i=0; i<500; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
