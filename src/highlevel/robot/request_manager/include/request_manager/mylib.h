#pragma once //avoid redefining the class when included somewhere else

#include <iostream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <sys/stat.h>
#include <stdlib.h>
#include <time.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <boost/date_time/posix_time/posix_time.hpp>



using namespace std;




/***************************** DATA STRUCTURES ********************************/




/************************  STRING MANIPULATION  *******************************/

// trim from start
static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}


//this function split the string and returns a vector
std::vector<std::string> split(std::string thestring, char separator) {
    std::vector<std::string> vec;
    std::istringstream f(thestring);
    std::string s;
    while (std::getline(f, s, separator)) {
        //std::cout << s << std::endl;
        vec.push_back(s);
    }

    return vec;
}




//*********************************  FILES  ***********************************
/**
 * Check if a file exists
 * @param[in] filename - the name of the file to check
 * @return    true if the file exists, else false
 */
bool fileExists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}





//*****************************  LOCATIONS ************************************

/**
 * This functions reads the a string of locations and generates a vector of 
 * vector
 * @param list
 * @param coordinates
 */
void addlocation(vector< vector<double> >& list, std::string coordinates){

    //Get the coordinates
    //printf("\n%s",coordinates.c_str());
    std::vector<std::string> stringvector = split(coordinates,',');
    //int vectorsize = stringvector.size();
    //printf("\nString size: %i",vectorsize);

    //Add the points to a list
    std::vector<double> coordinatesvector;
    for(int i=0; i<stringvector.size(); ++i){
        std::string point = stringvector.at(i);
        double dpoint = atof(point.c_str());
        coordinatesvector.push_back(dpoint);

    }

    //add the new coordinate to the list
    list.push_back(coordinatesvector);

}







// ***********************   TIME MANAGEMENT  *********************************


/**
 * Format current time (calculated as an offset in current day) in this form:
 * "hh:mm:ss.SSS" (where "SSS" are milliseconds)
 **/
string now_str()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now = 
        boost::posix_time::microsec_clock::local_time();

    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();

    //
    // Extract hours, minutes, seconds and milliseconds.
    //
    // Since there is no direct accessor ".milliseconds()",
    // milliseconds are computed _by difference_ between total milliseconds
    // (for which there is an accessor), and the hours/minutes/seconds
    // values previously fetched.
    //
    const long hours        = td.hours();
    const long minutes      = td.minutes();
    const long seconds      = td.seconds();
    const long milliseconds = td.total_milliseconds() -
                              ((hours * 3600 + minutes * 60 + seconds) * 1000);

    //
    // Format like this:
    //
    //      hh:mm:ss.SSS
    //
    // e.g. 02:15:40:321
    //
    //      ^          ^
    //      |          |
    //      123456789*12
    //      ---------10-     --> 12 chars + \0 --> 13 chars should suffice
    //  
    // 
    char buf[40];
    sprintf(buf, "%02ld:%02ld:%02ld.%03ld", 
        hours, minutes, seconds, milliseconds);

    return buf;
}


/**
 * Print the current time, including milliseconds
 */
void printcurrenttime(){
    /*
    struct tm *current;
    time_t now;

    time(&now);
    current = localtime(&now);

    printf("the time is %i:%i:%i\n", current->tm_hour, current->tm_min, current->tm_sec);
     */
    cout << now_str() << '\n';
    
}




