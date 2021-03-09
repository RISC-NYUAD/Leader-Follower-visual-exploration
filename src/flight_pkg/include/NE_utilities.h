#ifndef NE_UTILS
#define NE_UTILS

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <chrono>
#include <pwd.h>
#include <string>
#include <cmath>
#include <cstring>
#include <vector>
#include <stdlib.h>
#include <inttypes.h>
#include <sstream>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/format.hpp>
#include <iostream>

//Keyboard function used to terminate the program on any keyboard button hit
int kbhit(void);

//Function used to get user name
std::string get_username(void);

//Function used to return string type file name using current date/time
std::string date_filename(void);

//get distance 3D
double get_distance3D(double, double, double, double, double, double);

//get distance 2D
double get_distance2D(double, double, double, double);

double seconds_from_epoch(boost::posix_time::ptime const&);

inline double deg2rad(double x) {return x*M_PI/180;}
inline double rad2deg(double x) {return x*180/M_PI;}

#endif
