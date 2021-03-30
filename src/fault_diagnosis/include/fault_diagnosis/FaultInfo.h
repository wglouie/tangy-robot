#ifndef FAULT_INFO
#define FAULT_INFO

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <string>
#include <algorithm>
//#include <map>
#include "fault_diagnosis/Fault.h"

#define FAULT_LIST_LEN 3


#endif //FAULT_INFO

/*msg structure:
Header header (.stamp.sec to get time)
string monitor_name
string behaviour
string error_code
string fault_msg
int32 criticality
int32w technicality*/