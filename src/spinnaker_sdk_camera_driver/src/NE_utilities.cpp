//
#include "spinnaker_sdk_camera_driver/NE_utilities.h"


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

//get distance 3D
double get_distance3D(double x1, double y1, double z1, double x2, double y2, double z2){
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2));
}

//get distance 2D
double get_distance2D(double x1, double y1, double x2, double y2){
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

double seconds_from_epoch(boost::posix_time::ptime const& t)
{
    boost::posix_time::ptime const EPOCH(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration delta(t - EPOCH);
    return (delta.total_microseconds() / 1000000.0);
}
