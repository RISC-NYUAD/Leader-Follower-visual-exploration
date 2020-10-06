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
#include <iostream>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
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


#define PORT 9400
#define MAXLINE 200
#define num_objects 1

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
	std::string filename = std::string("exp1") + "/" + temp + "_" + std::to_string(parts->tm_year-100) //Set Year
											+ "m" + std::to_string(parts->tm_mon+1) //Set Month
												+ "d" + std::to_string(parts->tm_mday) //Set day
													+ "h" + std::to_string(parts->tm_hour) //Set hour
														+ "min" + std::to_string(parts->tm_min) //Set minute
															//+ "s" + std::to_string(parts->tm_sec) //Set second
																+ ".csv";
	return filename;
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
							{-7.5, 2.5},
							{7.5, 2.5},
							{7.5, -2.5},
							{-7.5, -2.5},
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
        }
     }

}


// Driver code 
int main(int argc, char **argv) { 

  if((sizeof(object_names)/sizeof(std::string))!=num_objects){
    ROS_INFO("Number of object names not equal to predefined num_objects. Exiting....");
    return -1;
  }
    ros::init(argc, argv, "client");

    ros::NodeHandle n("~");  

    ros::Subscriber Gapter_2_measurements = n.subscribe("/vrpn_client_node/"+ object_names[0] + "/pose", 1, &GapterPoseFeedback);

    ros::Rate loop_rate(1);

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

    while (kbhit()==0 && ros::ok())
    {
      
      ros::spinOnce();

      loop_rate.sleep();
    }
	std:: cout << "Data logged to " << fileName_vicon << std::endl;
	std:: cout << "Data logged to " << fileName_adhoc << std::endl;
	outfile_vicon.close();
    outfile_adhoc.close();
    close(sockfd);
      
    return 0; 
} 
