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

  /*
  printf("%.16lf\n", transX);
  printf("%.16lf\n", transY);
  printf("%.16lf\n", transZ);
  printf("%.16lf\n", quatW);
  printf("%.16lf\n", quatX);
  printf("%.16lf\n", quatY);
  printf("%.16lf\n", quatZ);
  */

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

// Driver code 
int main(int argc, char **argv) { 

  if((sizeof(object_names)/sizeof(std::string))!=num_objects){
    ROS_INFO("Number of object names not equal to predefined num_objects. Exiting....");
    return -1;
  }
    //srand (time (NULL));
    ros::init(argc, argv, "server");

    ros::NodeHandle n("~");

    ros::Subscriber Gapter_2_measurements = n.subscribe("/vrpn_client_node/"+ object_names[0] + "/pose", 1, &GapterPoseFeedback);

    ros::Rate loop_rate(100);

    int sockfd; 

    char buffer[MAXLINE]; 

    double prevTime = 0, deltaT = 0;

    stream.str(std::string());
    stream << std::fixed << std::setprecision(16) << transX << ", " << transY << ", " << transZ << ", " << quatW << ", " << quatX << ", " << quatY << ", " << quatZ << ", " << getNanosecTime();

    struct sockaddr_in servaddr; 
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    memset(&servaddr, 0, sizeof(servaddr)); 
      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
  	servaddr.sin_addr.s_addr = inet_addr("192.168.0.100"); //change

   // servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
      
    unsigned int len;
  
    len = sizeof(servaddr);  
  
    const char delim[2] = ",";

    int other_pose_num = 8;

    double other_pose[other_pose_num]; 

    int counter = 0;

    char *token;

    char *tmp_ptr;

    long long msg_counter_sen = 1;
    long long msg_counter_recv = 1;
    
    int n_size1 = 0;
    int n_size = 0;

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
      
      std::string s = stream.str();

      n_size = s.length(); 

      char message[n_size+1];

      strcpy(message, s.c_str()); 

      /*
      sendto(sockfd, (const char *)message, strlen(message),  
          0, (const struct sockaddr *) &cliaddr, 
          sizeof(cliaddr)); 
          */

      sendto(sockfd, (const char *)message, strlen(message),  
          0, (const struct sockaddr *) &servaddr, 
          len); 

      //printf("\nMessage %lld sent.\n", msg_counter_sen);  

      msg_counter_sen++;
     

//receive
      bzero(buffer, MAXLINE); 
      bzero(other_pose, other_pose_num); 

      /*
      n_size1 = recvfrom(sockfd, (char *)buffer, sizeof(buffer), 0, ( struct sockaddr *) &cliaddr, 
          &len); 
          */
      
      n_size1 = recvfrom(sockfd, (char *)buffer, MAXLINE,  
          MSG_DONTWAIT, ( struct sockaddr *) &servaddr, 
          &len); 

      /*
      n_size1 = recvfrom(sockfd, (char *)buffer, MAXLINE,  
          MSG_DONTWAIT, ( struct sockaddr *) &cliaddr, 
          &len); 
      */

      if(n_size1 > 0)
      {
        //printf("error: %s\n", strerror(errno));
        
        //printf("%d\n", n_size1);

        //buffer[n_size1] = '\0'; 

        token = strtok(buffer, delim);

        counter = 0;

        while( token != NULL ) 
        {
          other_pose[counter] = strtod(token, &tmp_ptr);
          counter++;
          token = strtok(NULL, delim);
        }
        

/*
	    //Get current date/time
	    auto now = std::chrono::system_clock::now();
	    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	    struct tm *parts = std::localtime(&now_c);
        std::string hours_t = std::to_string(parts->tm_hour);
        std::string minutes_t = std::to_string(parts->tm_min);
        std::string seconds_t = std::to_string(parts->tm_sec);
        double timeElapsed = (((atoi(hours_t.c_str()))+4)*3600 + atoi(minutes_t.c_str())*60 + atoi(seconds_t.c_str()))*1000000000;

*/

        deltaT = (other_pose[7] - prevTime);

        if(other_pose[0] < 1000000) //experiment begun??
        {
		    outfile_adhoc << std::scientific << std::setprecision(16) << other_pose[7] << ";" << other_pose[0] << ";" << other_pose[1] << ";"<< other_pose[2] << ";"
				    << other_pose[3] << ";" << other_pose[4] << ";" << other_pose[5] << ";"<< other_pose[6] ;
		    outfile_adhoc << std::endl;

            prevTime = other_pose[7];

            //printf("DeltaT: %f\n", deltaT);
            

            for(int i = 0; i < other_pose_num; i++)
            {
              //printf("%.16lf\n", other_pose[i]);
            }

            //printf("\n");  
            //printf("Message %lld received.\n", msg_counter_recv);  
            //printf("\n-----------------------------------------------------------\n");

            msg_counter_recv++;  
        }
     }

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
