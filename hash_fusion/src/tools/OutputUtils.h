#ifndef __OUTPUT_UTILS__
#define __OUTPUT_UTILS__

#include <ros/ros.h>

namespace output {	
	
	extern const char* format_white;
	extern const char* format_red;
	extern const char* format_green;
	extern const char* format_yellow;
	extern const char* format_blue;
	extern const char* format_purple;

	extern const char* format_yes;
	extern const char* format_no;
}

#define ROS_INFO_PURPLE(str, ...) ROS_INFO("\033[35m"#str"\033[0m", __VA_ARGS__)

#endif