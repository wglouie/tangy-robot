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



/************************  CALLING EXTERNAL PROGRAMS  *************************/

/**
 * This function executes a command and gets the output stream as a string
 * @param cmd
 * @return 
 */
std::string exec(char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}



/**
 * This function runs a command and don't wait for it.
 */
#define READ 0
#define WRITE 1
pid_t popen2(char **command, int *infp, int *outfp)
{
    int p_stdin[2], p_stdout[2];
    pid_t pid;

    if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0)
        return -1;

    pid = fork();

    if (pid < 0)
        return pid;
    else if (pid == 0)
    {
        close(p_stdin[WRITE]);
        dup2(p_stdin[READ], READ);
        close(p_stdout[READ]);
        dup2(p_stdout[WRITE], WRITE);

        execvp(*command, command);
        perror("execvp");
        exit(1);
    }

    if (infp == NULL)
        close(p_stdin[WRITE]);
    else
        *infp = p_stdin[WRITE];

    if (outfp == NULL)
        close(p_stdout[READ]);
    else
        *outfp = p_stdout[READ];

    return pid;
}


/**
 * This function get the process id (pid) of the first process with the given 
 * name procName
 * @param actionstr
 * @return 
 */
int getProcIdByName(string procName)
{
    int pid = -1;

    // Open the /proc directory
    DIR *dp = opendir("/proc");
    if (dp != NULL)
    {
        // Enumerate all entries in directory until process found
        struct dirent *dirp;
        while (pid < 0 && (dirp = readdir(dp)))
        {
            // Skip non-numeric entries
            int id = atoi(dirp->d_name);
            if (id > 0)
            {
                // Read contents of virtual /proc/{pid}/cmdline file
                string cmdPath = string("/proc/") + dirp->d_name + "/cmdline";
                ifstream cmdFile(cmdPath.c_str());
                string cmdLine;
                getline(cmdFile, cmdLine);
                if (!cmdLine.empty())
                {
                    // Keep first cmdline item which contains the program path
                    size_t pos = cmdLine.find('\0');
                    if (pos != string::npos)
                        cmdLine = cmdLine.substr(0, pos);
                    // Keep program name only, removing the path
                    pos = cmdLine.rfind('/');
                    if (pos != string::npos)
                        cmdLine = cmdLine.substr(pos + 1);
                    // Compare against requested process name
                    if (procName == cmdLine)
                        pid = id;
                }
            }
        }
    }

    closedir(dp);

    return pid;
}






/***************************  READING PLAN INPUTS  ****************************/

/**
 * This function creates a vector of string for each element of an action string
 * For example, an action '0: MOVE R1 A B' will result in a vector with 5 
 * elements.
 * @param actionstr
 * @return 
 */
std::vector<std::string> getactionvector(std::string actionstr) {
    std::vector<std::string> actionvector = split(actionstr,' ');

    return actionvector;
}



/**
 * This functions reads the console output from the planner and generates a 
 * vectors of actions (designed for metric-ff)
 * @param output
 * @return 
 */
std::vector< std::vector<std::string> > getplan(std::string output) {

    std::vector< std::vector<std::string> > list;
    list.clear();
        
    std::istringstream f(output);
    std::string line;
    bool planstarted = false;
    bool planended = false;
    while (std::getline(f, line)) {
        std::string planline = trim(line);
        if (!planstarted){

            //identify first line of the plan
            int found = planline.find("step");
            if (!planline.empty() and found != std::string::npos){
                planline.erase(found,4); //delete the word step: 4 letters
                list.push_back(getactionvector(trim(planline)));
                planstarted = true;
            }

        }
        else{
            if (!planline.empty()){
                list.push_back(getactionvector(planline));
            }else{
                planended = true;
                break;
            }
        }
    }
    return list;
}


/**
 * This function reads the console output from the planner and gets the planner informations
 */
/**
 * This functions reads the console output from the planner and gets the 
 * strings concerning the plan information only, including stats and actions. 
 * The marker identifies where the plan info starts.
 * @param output
 * @param marker
 * @param markerunsovable
 * @return 
 */
std::string getplanstring(std::string output, std::string marker) {

    std::string planstr = "";
    
    //printf("%s\n",output.c_str());
        
    std::istringstream f(output);
    std::string line;
    bool planstarted = false;
    bool planended = false;
    while (std::getline(f, line)) {
        std::string planline = trim(line);
        //printf("%s\n",planline.c_str());
        if (!planstarted){

            //identify the marker to identify the beginning of the plan info
            // which will be in the next line.
            int found = planline.find(marker);
            if (!planline.empty() and found != std::string::npos){
                //printf("Got it: %s\n\n",planline.c_str());
                planstarted = true;
            }

        }
        else{
            if (planline != ""){
                planstr += planline + "\n";
            }else{
                planended = true;
                break;
            }
        }
    }    
    //printf("%s\n",planstr.c_str());
    return planstr;
}




/**
 * Print the plan
 * @param plan
 */
void printPlan(vector< vector<string> > plan){
    //Print the plan
    for(int i=0; i<plan.size(); ++i){
        vector<string> actionvec = plan.at(i);
        for(int j=0; j<actionvec.size(); ++j){
            cout<< actionvec.at(j) + ' ';
        }
        cout<< '\n';
    }
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




