#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <time.h>


#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>
#include <std_msgs/UInt32.h>
#include <fstream>
#include <thread>
#include <ios>
#include <cstring>
#include <chrono> //ctime is included
#include <vector>
#include <stdlib.h>
#include <inttypes.h>
#include <sstream>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <pwd.h>
#include <unistd.h>
#include <sys/types.h>
#include <experimental/filesystem>


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

using namespace std;

#define PORT 9400
#define MAXLINE 200
#define num_objects 1

#define takeoff_alt 0.3
#define rect_size 1.5

#define NUM_POINTS 1

#define RATE 25
//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped initial_pose, current_pose, target_pose;
double current_heading; //rad
bool stop_exec=false;
float GYM_OFFSET;

float tollerance = .10;

std::ofstream outfile;

float points[NUM_POINTS][4]={{0.0, rect_size, takeoff_alt, 0}};

//double transX, transY, transZ, quatX, quatY, quatZ, quatW;

std::stringstream stream;

double transX = 1000000;
double transY = 2000000;
double transZ = 3000000;

double quatX = 4000000;
double quatY = 5000000;
double quatZ = 6000000;
double quatW = 7000000;

int ch;

geometry_msgs::Pose Gapter_2; //other drone

bool flag = 1;

std::string object_names[num_objects]={"Gapter_2"}; //change

std::ofstream outfile_vicon;
std::ofstream outfile_adhoc;
/*
long getNanosecTime()
{
    //Get current date/time
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    struct tm *parts = std::localtime(&now_c);
    std::string hours_t = std::to_string(parts->tm_hour);
    std::string minutes_t = std::to_string(parts->tm_min);
    std::string seconds_t = std::to_string(parts->tm_sec);
    long timeElapsed = (((atoi(hours_t.c_str()))+4)*3600 + atoi(minutes_t.c_str())*60 + atoi(seconds_t.c_str()))*1000000000;

    return timeElapsed;
}
*/

//get distance 3D
double get_distance3D(double x1, double y1, double z1, double x2, double y2, double z2){
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2));
}

//get distance 2D
double get_distance2D(double x1, double y1, double x2, double y2){
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

//Keyboard function used to terminate the program on user input
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
	
	ch = getchar();
	
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	
	if (ch != EOF)
	{
	  return 1;
	}
	return 0;
}

double getNanosecTime()
{
    double secsNow = ros::Time::now().toSec();
    return secsNow;
}

//Function used to return string type file name using current date/time
std::string date_filename(std::string temp){
	//Get current date/time
	auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	struct tm *parts = std::localtime(&now_c);
	//Assign 
	std::string filename = temp + "_" + std::to_string(parts->tm_year-100) //Set Year
											+ "m" + std::to_string(parts->tm_mon+1) //Set Month
												+ "d" + std::to_string(parts->tm_mday) //Set day
													+ "h" + std::to_string(parts->tm_hour) //Set hour
														+ "min" + std::to_string(parts->tm_min) //Set minute
															//+ "s" + std::to_string(parts->tm_sec) //Set second
																+ ".csv";
	return filename;
}

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

//stop execution
void stop_cb(const std_msgs::Bool input)
{
  ROS_INFO("Start stop data received");
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
  current_pose = *msg;
  getHeading(current_pose.pose);
}
 
  const char delim[2] = ",";

  int other_pose_num = 8;

  double other_pose[8]; 

  int counter = 0;

  char *token;

  char *tmp_ptr;

  long long msg_counter_sen = 1;
  long long msg_counter_recv = 1;
  
  int n_size1 = 0;
  int n_size = 0;

  int sockfd; 

  char buffer[MAXLINE]; 

  double prevTime = 0, deltaT = 0;

struct sockaddr_in servaddr; 
    
  unsigned int len;

struct Point2D
{
	double x;
	double y;
};


double getslope(double x1, double y1, double x2, double y2) {
	return (y2 - y1) / (x2 - x1);
}

double getyint(double x1, double y1, double x2, double y2) {
	double m = (y2 - y1) / (x2 - x1);
	return (y1 - (m * x1));
}

double getyint_2(double x1, double y1, double m) {
	return (y1 - (m * x1));
}

int get_intersection(double m, double b, const double edg[4][2], double points[2][2]) {
	int inter_case = 0;
	int case_number = 0;
	int idx = 0;
	double test_x = 0;
	double test_y = edg[0][1];
	test_x = (test_y - b) / m;
	if (edg[0][0] <= test_x && edg[1][0] >= test_x) {
		points[idx][0] = test_x;
		points[idx][1] = test_y;
		idx++;
		inter_case++;
	}
	test_x = edg[1][0];
	test_y = m * test_x + b;
	if (edg[2][1] < test_y && edg[1][1] > test_y) {
		points[idx][0] = test_x;
		points[idx][1] = test_y;
		idx++;
		inter_case = inter_case + 2;
	}
	test_y = edg[2][1];
	test_x = (test_y - b) / m;
	if (edg[3][0] <= test_x && edg[2][0] >= test_x) {
		points[idx][0] = test_x;
		points[idx][1] = test_y;
		idx++;
		inter_case = inter_case + 4;
	}
	test_x = edg[0][0];
	test_y = m * test_x + b;
	if (edg[3][1] < test_y && edg[0][1] > test_y) {
		points[idx][0] = test_x;
		points[idx][1] = test_y;
		idx++;
		inter_case = inter_case + 8;
	}

	// Assign number value to each of the possible cases 
	if (inter_case == 3)
		case_number = 1; // edge 1 and 2 have intersections
	if (inter_case == 5)
		case_number = 2; // edge 1 and 3 have intersections
	if (inter_case == 9)
		case_number = 3; // edge 1 and 4 have intersections
	if (inter_case == 6)
		case_number = 4; // edge 2 and 3 have intersections
	if (inter_case == 10)
		case_number = 5; // edge 2 and 4 have intersections
	if (inter_case == 12)
		case_number = 6; // edge 3 and 4 have intersections
	std::cout << "case number: " << case_number << std::endl;
	return case_number;
}

Point2D compute2DPolygonCentroid(Point2D* vertices, int vertexCount)
{
	Point2D centroid = { 0, 0 };
	double signedArea = 0.0;
	double x0 = 0.0; // Current vertex X
	double y0 = 0.0; // Current vertex Y
	double x1 = 0.0; // Next vertex X
	double y1 = 0.0; // Next vertex Y
	double a = 0.0;  // Partial signed area

	// For all vertices
	int i = 0;
	for (i = 0; i < vertexCount; ++i)
	{
		x0 = vertices[i].x;
		y0 = vertices[i].y;
		x1 = vertices[(i + 1) % vertexCount].x;
		y1 = vertices[(i + 1) % vertexCount].y;
		a = x0 * y1 - x1 * y0;
		signedArea += a;
		centroid.x += (x0 + x1) * a;
		centroid.y += (y0 + y1) * a;
	}

	signedArea *= 0.5;
	centroid.x /= (6.0 * signedArea);
	centroid.y /= (6.0 * signedArea);

	return centroid;
}

void GapterPoseFeedback(geometry_msgs::PoseStamped VulcanPose_){
  if(flag)
  {
        
  flag = false;

  Gapter_2=VulcanPose_.pose;
  transX = Gapter_2.position.x; 
  transY = Gapter_2.position.y; 
  transZ = Gapter_2.position.z;
  quatW = Gapter_2.orientation.w; 
  quatX = Gapter_2.orientation.x;
  quatY = Gapter_2.orientation.y;
  quatZ = Gapter_2.orientation.z;

  double timeNow = getNanosecTime();

  stream.str(std::string());
  stream << std::fixed << std::setprecision(16) << transX << ", " << transY << ", " << transZ << ", " << quatW << ", " << quatX << ", " << quatY << ", " << quatZ << ", " << timeNow;
  
		outfile_vicon << std::scientific << std::setprecision(16) << getNanosecTime() << ";" << Gapter_2.position.x << ";" << Gapter_2.position.y << ";"<< Gapter_2.position.z << ";"
				<< Gapter_2.orientation.w << ";" << Gapter_2.orientation.x << ";" << Gapter_2.orientation.y << ";"<< Gapter_2.orientation.z ;
		outfile_vicon << std::endl;
    

     std::string s = stream.str();

      n_size = s.length(); 

      char message[n_size+1];

      strcpy(message, s.c_str()); 

      sendto(sockfd, (const char *)message, strlen(message),  
          0, (const struct sockaddr *) &servaddr, 
          len); 

      msg_counter_sen++;
     

//receive
      bzero(buffer, MAXLINE); 
      bzero(other_pose, other_pose_num); 

      n_size1 = recvfrom(sockfd, (char *)buffer, MAXLINE,  
          MSG_DONTWAIT, ( struct sockaddr *) &servaddr, 
          &len); 

      if(n_size1 > 0)
      {
        token = strtok(buffer, delim);

        counter = 0;

        while( token != NULL ) 
        {
          other_pose[counter] = strtod(token, &tmp_ptr);
          counter++;
          token = strtok(NULL, delim);
        }

        deltaT = (other_pose[7] - prevTime);

        if(other_pose[0] < 1000000) //experiment begun??
        {
		    outfile_adhoc << std::scientific << std::setprecision(16) << other_pose[7] << ";" << other_pose[0] << ";" << other_pose[1] << ";"<< other_pose[2] << ";"
				    << other_pose[3] << ";" << other_pose[4] << ";" << other_pose[5] << ";"<< other_pose[6] ;
		    outfile_adhoc << std::endl;

            prevTime = other_pose[7];

            for(int i = 0; i < other_pose_num; i++)
            {
              //printf("%.16lf\n", other_pose[i]);
            }

            //printf("\n");  
            //printf("Message %lld received.\n", msg_counter_recv);  
            //printf("\n-----------------------------------------------------------\n");

            msg_counter_recv++;  

            //voronoi stuff
            
	        const double y1 = transX;       //Add coordinates of drones here
	        const double x1 = transX;  //Add coordinates of drones here
	        const double y2 = other_pose[0];        //Add coordinates of drones here
	        const double x2 = other_pose[1]; //Add coordinates of drones here

                printf("%.16lf\n", transX);
                printf("%.16lf\n", transY);
            	printf("\n");  

        	//Four vertices (x,y) of the arena (clockwise from top left)
            const double edg[4][2] = {
							{-2.5, 7.5},
							{2.5, 7.5},
							{2.5, -7.5},
							{-2.5, -7.5},
	                };

            //get equation of line
	        double m1 = getslope(x1, y1, x2, y2);
	        double y_int_1 = getyint(x1, y1, x2, y2);
	        std::cout << "Equation of line: " << "y =" << m1 << " x+" << y_int_1 << std::endl;

	        //get equation of the perpendicular line
	        double  m2;
	        
	        //Slope is zero
	        if (m1==0) {
	          m2=9999999999;
	        }
	        //Otherwise
	        else {
	          m2 = -(1 / m1);
	        }
	        
	        double midpoint_x = (x1 + x2) / 2;
	        double midpoint_y = (y1 + y2) / 2;
	        double y_int_2 = getyint_2(midpoint_x, midpoint_y, m2);
	        std::cout << "Equation of the perp line: " << "y =" << m2 << " x+" << y_int_2 << std::endl;


	        double inter_points[2][2] = { {0, 0}, {0, 0} };
	        int centroid_case = 0;
	        centroid_case = get_intersection(m2, y_int_2, edg, inter_points);
	        std::cout << "intersection: " << inter_points[0][0] << "," << inter_points[0][1] << std::endl;
	        std::cout << "intersection: " << inter_points[1][0] << "," << inter_points[1][1] << std::endl;

	        Point2D centroid_1 = { 0, 0 };
	        Point2D centroid_2 = { 0, 0 };
	        Point2D vertices_1[5];
	        Point2D vertices_2[5];

	        switch (centroid_case) {
	        case 1:
		        // Assign vertices_1 in order 
		        vertices_1[0].x = edg[0][0];
		        vertices_1[0].y = edg[0][1];

		        vertices_1[1].x = inter_points[0][0];
		        vertices_1[1].y = inter_points[0][1];

		        vertices_1[2].x = inter_points[1][0];
		        vertices_1[2].y = inter_points[1][1];

		        vertices_1[3].x = edg[2][0];
		        vertices_1[3].y = edg[2][1];

		        vertices_1[4].x = edg[3][0];
		        vertices_1[4].y = edg[3][1];

		        //Assign vertices_2 in order
		        vertices_2[0].x = inter_points[0][0];
		        vertices_2[0].y = inter_points[0][1];

		        vertices_2[1].x = edg[1][0];
		        vertices_2[1].y = edg[1][1];

		        vertices_2[2].x = inter_points[1][0];
		        vertices_2[2].y = inter_points[1][1];

		        //Calculate centroids
		        centroid_1 = compute2DPolygonCentroid(vertices_1, 5);
		        centroid_2 = compute2DPolygonCentroid(vertices_2, 3);
		        break;
	        case 2:
		        // Assign vertices_1 in order 
		        vertices_1[0].x = edg[0][0];
		        vertices_1[0].y = edg[0][1];

		        vertices_1[1].x = inter_points[0][0];
		        vertices_1[1].y = inter_points[0][1];

		        vertices_1[2].x = inter_points[1][0];
		        vertices_1[2].y = inter_points[1][1];

		        vertices_1[3].x = edg[3][0];
		        vertices_1[3].y = edg[3][1];


		        //Assign vertices_2 in order
		        vertices_2[0].x = inter_points[0][0];
		        vertices_2[0].y = inter_points[0][1];

		        vertices_2[1].x = edg[1][0];
		        vertices_2[1].y = edg[1][1];

		        vertices_2[2].x = edg[2][0];
		        vertices_2[2].y = edg[2][1];

		        vertices_2[3].x = inter_points[1][0];
		        vertices_2[3].y = inter_points[1][1];

		        //Calculate centroids
		        centroid_1 = compute2DPolygonCentroid(vertices_1, 4);
		        centroid_2 = compute2DPolygonCentroid(vertices_2, 4);
		        break;
	        
	        case 3:
		        // Assign vertices_1 in order 
		        vertices_1[0].x = edg[0][0];
		        vertices_1[0].y = edg[0][1];

		        vertices_1[1].x = inter_points[0][0];
		        vertices_1[1].y = inter_points[0][1];

		        vertices_1[2].x = inter_points[1][0];
		        vertices_1[2].y = inter_points[1][1];

		        //Assign vertices_2 in order
		        vertices_2[0].x = inter_points[0][0];
		        vertices_2[0].y = inter_points[0][1];

		        vertices_2[1].x = edg[1][0];
		        vertices_2[1].y = edg[1][1];

		        vertices_2[2].x = edg[2][0];
		        vertices_2[2].y = edg[2][1];

		        vertices_2[3].x = edg[3][0];
		        vertices_2[3].y = edg[3][1];

		        vertices_2[4].x = inter_points[1][0];
		        vertices_2[4].y = inter_points[1][1];

		        //Calculate centroids
		        centroid_1 = compute2DPolygonCentroid(vertices_1, 3);
		        centroid_2 = compute2DPolygonCentroid(vertices_2, 5);
		        break;
	        case 4:
		        // Assign vertices_1 in order 
		        vertices_1[0].x = edg[0][0];
		        vertices_1[0].y = edg[0][1];

		        vertices_1[1].x = edg[1][0];
		        vertices_1[1].y = edg[1][1];

		        vertices_1[2].x = inter_points[0][0];
		        vertices_1[2].y = inter_points[0][1];

		        vertices_1[3].x = inter_points[1][0];
		        vertices_1[3].y = inter_points[1][1];

		        vertices_1[4].x = edg[3][0];
		        vertices_1[4].y = edg[3][1];

		        //Assign vertices_2 in order
		        vertices_2[0].x = inter_points[0][0];
		        vertices_2[0].y = inter_points[0][1];

		        vertices_2[1].x = edg[2][0];
		        vertices_2[1].y = edg[2][1];

		        vertices_2[2].x = inter_points[1][0];
		        vertices_2[2].y = inter_points[1][1];

		        //Calculate centroids
		        centroid_1 = compute2DPolygonCentroid(vertices_1, 5);
		        centroid_2 = compute2DPolygonCentroid(vertices_2, 3);
		        break;
	        case 5:
		        // Assign vertices_1 in order 
		        vertices_1[0].x = edg[0][0];
		        vertices_1[0].y = edg[0][1];

		        vertices_1[1].x = edg[1][0];
		        vertices_1[1].y = edg[1][1];

		        vertices_1[2].x = inter_points[0][0];
		        vertices_1[2].y = inter_points[0][1];

		        vertices_1[3].x = inter_points[1][0];
		        vertices_1[3].y = inter_points[1][1];

		        //Assign vertices_2 in order
		        vertices_2[0].x = inter_points[1][0];
		        vertices_2[0].y = inter_points[1][1];

		        vertices_2[1].x = inter_points[0][0];
		        vertices_2[1].y = inter_points[0][1];

		        vertices_2[2].x = edg[2][0];
		        vertices_2[2].y = edg[2][1];

		        vertices_2[3].x = edg[3][0];
		        vertices_2[3].y = edg[3][1];

		        //Calculate centroids
		        centroid_1 = compute2DPolygonCentroid(vertices_1, 4);
		        centroid_2 = compute2DPolygonCentroid(vertices_2, 4);

		        break;
	        case 6:
		        // Assign vertices_1 in order 
		        vertices_1[0].x = edg[0][0];
		        vertices_1[0].y = edg[0][1];

		        vertices_1[1].x = edg[1][0];
		        vertices_1[1].y = edg[1][1];

		        vertices_1[2].x = edg[2][0];
		        vertices_1[2].y = edg[2][1];

		        vertices_1[3].x = inter_points[0][0];
		        vertices_1[3].y = inter_points[0][1];

		        vertices_1[4].x = inter_points[1][0];
		        vertices_1[4].y = inter_points[1][1];

		        //Assign vertices_2 in order
		        vertices_2[0].x = inter_points[1][0];
		        vertices_2[0].y = inter_points[1][1];

		        vertices_2[1].x = inter_points[0][0];
		        vertices_2[1].y = inter_points[0][1];

		        vertices_2[2].x = edg[2][0];
		        vertices_2[2].y = edg[2][1];

		        //Calculate centroids
		        centroid_1 = compute2DPolygonCentroid(vertices_1, 5);
		        centroid_2 = compute2DPolygonCentroid(vertices_2, 3);
		        break;
	        default:
		        break;
	        }

	        std::cout << "Final centroid 1: (" << centroid_1.x << "," << centroid_1.y << ")" << std::endl;
	        std::cout << "Final centroid 2: (" << centroid_2.x << "," << centroid_2.y << ")" << std::endl;

            points[0][0] = (float) (centroid_1.x - transX);
            points[0][1] = (float) (centroid_1.y - transY);
            points[0][2] = takeoff_alt;
            points[0][3] = 0;
        }
        }
     }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "staple_rect_indoor_1");
  ros::NodeHandle nh;
  ROS_INFO("Start");
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(RATE);
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  ros::Publisher stop_execution_pub = nh.advertise<std_msgs::Bool>("/stop_execution", 10);
  ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
  ros::Subscriber Gapter_2_measurements = nh.subscribe("/vrpn_client_node/"+ object_names[0] + "/pose", 1, &GapterPoseFeedback);
  ros::Subscriber stop_execution = nh.subscribe<std_msgs::Bool>("/startstop_rectangle", 10, stop_cb);

    stream.str(std::string());
    stream << std::fixed << std::setprecision(16) << transX << ", " << transY << ", " << transZ << ", " << quatW << ", " << quatX << ", " << quatY << ", " << quatZ << ", " << getNanosecTime();
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    memset(&servaddr, 0, sizeof(servaddr)); 
      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
  	servaddr.sin_addr.s_addr = inet_addr("192.168.0.102"); //change

   // servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
        
    len = sizeof(servaddr);  

    std::string fileName_vicon = date_filename("vicon");
    std::string fileName_adhoc = date_filename("adhoc");

	outfile_vicon.open(fileName_vicon, std::ios::out | std::ios::trunc);
    outfile_adhoc.open(fileName_adhoc, std::ios::out | std::ios::trunc);

    outfile_vicon.exceptions(outfile_vicon.exceptions() | std::ios::failbit | std::ifstream::badbit);
    outfile_adhoc.exceptions(outfile_adhoc.exceptions() | std::ios::failbit | std::ifstream::badbit);

	if (outfile_vicon.fail())
	{
		throw std::ios_base::failure(std::strerror(errno));
		return -1;
	}

	if (outfile_adhoc.fail())
	{
		throw std::ios_base::failure(std::strerror(errno));
		return -1;
	}

    outfile_vicon << "G2_time (MSec)"<< ";" << "Gapter_2 (t_x)" << ";" <<  "t_y" << ";" <<  "t_z" << ";"
		<< "Gapter_2 (q_w)" << ";" <<  "q_x" << ";" <<  "q_y" << ";" << "q_z" << ";" << std::endl;


    outfile_adhoc << "G1_time (MSec)"<< ";" << "Gapter_1 (t_x)" << ";" <<  "t_y" << ";" <<  "t_z" << ";"
		<< "Gapter_1 (q_w)" << ";" <<  "q_x" << ";" <<  "q_y" << ";" << "q_z" << ";" << std::endl;

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
  
//   Initialization - Log file
	std::string usr="odroid";
	std::string fileName;
	if(usr!="?") fileName="/home/"+usr+"/projects/flight_pkg/logs/"+date_filename("default"); //try to get username, if not save to home folder
	else fileName=date_filename("default");
	outfile.open(fileName, std::ios::out | std::ios::app);
	if (outfile.fail()){
		throw std::ios_base::failure(std::strerror(errno));
		return -1;
	}
// 
// 	make sure write fails with exception if something is wrong
	outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit);
	outfile << "time_elapsed(Sec)"<< ";" 
				<< "Gapter waypoint (X)" << ";" << "Gapter.waypoint (Y)" << ";" << "Gapter.waypoint (Z)" << ";" 
						<< "Gapter Position(x)" << ";" <<  "y" << ";" <<  "z" << std::endl;
  
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

  // Hover for 5 seconds
  ROS_INFO("HOVERING...");
  for(int i=0; i<500; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  initial_pose = current_pose;
  //Switch positions
  int wp=0;
  double waypoint_time_init = ros::Time::now().toSec();
  double dMag = 0;
  ROS_INFO("Waypoint %d",wp);
  while(ros::ok() && !stop_exec && kbhit()==0 && wp<NUM_POINTS){
        setHeading(points[wp][3]);
        setDestination(points[wp][0], points[wp][1], points[wp][2]);
        for (int i = 20*RATE; ros::ok() && i > 0 ; --i){
        local_pos_pub.publish(target_pose);
        dMag = get_distance3D(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
        //cout << "Mag: " <<  dMag << endl;
        outfile << ros::Time::now().toSec()-waypoint_time_init << ";" 
            << target_pose.pose.position.x << ";" << target_pose.pose.position.y << ";" << target_pose.pose.position.z << ";" 
                    << current_pose.pose.position.x << ";" <<  current_pose.pose.position.y << ";" <<  current_pose.pose.position.z << std::endl;
        if( dMag < tollerance) break;
        ros::spinOnce();
        rate.sleep();
        if(i == 1) ROS_INFO("Failed to reach destination. Stepping to next task.");
        }
        wp++;
        ROS_INFO("Waypoint %d",wp);
    }
  outfile.close();
  //while(ros::ok() && dMag<tollerance){
  //      ROS_INFO("Waypoint landing");
  //      setHeading(points[NUM_POINTS-1][3]);
  //      setDestination(points[NUM_POINTS-1][0], points[NUM_POINTS-1][1], points[NUM_POINTS-1][2]);
  //      for (int i = 20*RATE; ros::ok() && i > 0 ; --i){
  //      local_pos_pub.publish(target_pose);
  //      dMag = get_distance3D(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
  //      if( dMag < tollerance) break;
  //      ros::spinOnce();
  //      rate.sleep();
  //      if(i == 1) ROS_INFO("Failed to reach destination. Stepping to next task.");
  //      }
  //}
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
	std:: cout << "Data logged to " << fileName_vicon << std::endl;
	std:: cout << "Data logged to " << fileName_adhoc << std::endl;
	outfile_vicon.close();
    outfile_adhoc.close();
    close(sockfd);
  return 0;
}
