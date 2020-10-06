#include <iostream>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <ctime>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <pwd.h>

//Set global variables
sensor_msgs::NavSatFix GPSfix, GPSfix_raw;
sensor_msgs::TimeReference TimeReference;
geometry_msgs::PoseStamped localPose;
mavros_msgs::State currentState;

std::ofstream outfile;

//Function used to return string type file name using current date/time
std::string date_filename(void)
{
	//Get current date/time
	auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	struct tm *parts = std::localtime(&now_c);
	//Assign 
	std::string filename=               std::to_string(parts->tm_year-100) //Set Year
											+ "m" + std::to_string(parts->tm_mon+1) //Set Month
												+ "d" + std::to_string(parts->tm_mday) //Set day
													+ "h" + std::to_string(parts->tm_hour) //Set hour
														+ "min" + std::to_string(parts->tm_min) //Set minute
															//+ "s" + std::to_string(parts->tm_sec) //Set second
																+ ".csv";
	return filename;
}

//Keyboard function used to terminate the program on any keyboard button hit
int kbhit(void)
{
	struct termios oldt, newt;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	int ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)	return 1;
	return 0;
}

//Function used to get user name
std::string get_username() {
	struct passwd *pwd = getpwuid(getuid());
	if (pwd)	return pwd->pw_name;
	return "?";
}

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  currentState = *msg;
  bool connected = currentState.connected;
  bool armed = currentState.armed;
}

//get current position of drone
void GPSraw_cb(const sensor_msgs::NavSatFix::ConstPtr& NavSatFixRaw_)
{
	GPSfix_raw=*NavSatFixRaw_;
	//Logging
	outfile << TimeReference.time_ref.sec << ";" << TimeReference.time_ref.sec <<  ";" 
				<< GPSfix.latitude << ";" << GPSfix.longitude << ";" << GPSfix.altitude << ";"
					<< GPSfix_raw.latitude << ";" << GPSfix_raw.longitude << ";" << GPSfix_raw.altitude << ";"
						<< localPose.pose.position.x << ";" << localPose.pose.position.y << ";" << localPose.pose.position.z << std::endl;
}

//get current GPS position of drone
void GPS_cb(const sensor_msgs::NavSatFix::ConstPtr& NavSatFix_)
{
	GPSfix=*NavSatFix_;
}

//get local position output from EKF
void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& localPose_)
{
	localPose=*localPose_;
}

//get reference time (from external time source a.k.a. GPS -- see FCU documentation for BRD_RTC_TYPES)
void time_reference_cb(const sensor_msgs::TimeReference::ConstPtr TimeReference_)
{
	TimeReference=*TimeReference_;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "GPS_measurements");
	ros::NodeHandle nh;

	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
	ros::Subscriber GPS_raw = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 1, GPSraw_cb);
	ros::Subscriber GPS_global = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, GPS_cb);
	ros::Subscriber local_position = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, localPose_cb);
	ros::Subscriber current_time = nh.subscribe<sensor_msgs::TimeReference>("/mavros/time_reference", 1, time_reference_cb);

	// allow the subscribers to initialize
	ROS_INFO("INITIALIZING...");
	for(int i=0; i<100; i++)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	
	// wait for FCU connection
	ROS_INFO("CHECKING FCU CONNECTION...");
	while (ros::ok() && !currentState.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	//Initialization - Log file
	std::string usr=get_username();
	std::string fileName;
	if(usr!="?") fileName="/home/"+usr+"/projects/gapter_UAV/devel/lib/gps_measurements/"+date_filename(); //try to get username, if not save to home folder
	else fileName=date_filename();
	outfile.open(fileName, std::ios::out | std::ios::app);
	if (outfile.fail()){
		throw std::ios_base::failure(std::strerror(errno));
		return -1;
	}

	//make sure write fails with exception if something is wrong
	outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit);
	outfile << "time_ref(Sec)"<< ";" << "time_ref(NSec)" << ";" 
				<< "GPSfix.latitude" << ";" << "GPSfix.longitude" << ";" << "GPSfix.altitude" << ";" 
					<< "GPSfix_raw.latitude" << ";" << "GPSfix_raw.longitude" << ";" << "GPSfix_raw.altitude" << ";"
						<< "Local Position(x)" << ";" <<  "y" << ";" <<  "z" << std::endl;
	std::cout<< "Saving data to: " << fileName << std::endl;
	//MAIN LOOP
	while (ros::ok() && kbhit()==0)
	{
		ros::spinOnce();
		rate.sleep();
	}
	
	//Shutdown
	std::cout<< "Saved data to: " << fileName << std::endl;
	outfile.close();
	return 0;
}
