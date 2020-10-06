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

#endif
