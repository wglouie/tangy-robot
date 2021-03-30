//============================================================================
// Name        : drrobot_master_script_reader.cpp
// Author      : Ananias Paiva
// Copyright   : 
// Description : Read and execute differents scripts
//============================================================================

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "DrRobotMotionArmDriver.hpp"
#include "drrobot_script_teleop.hpp"
#include "drrobot_head_script.hpp"
#include <drrobot_h20_arm_player/ArmCmd.h>
#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <boost/thread.hpp>
#include <boost/timer.hpp>
#include <boost/bind.hpp>
#include <unistd.h>

//Include headers for ROS service
#include <actionlib/server/simple_action_server.h>
#include <script_player/ScriptPlayAction.h>
#include <string>

class DrRobotScriptReader
{
    private:
        
		ros::NodeHandle nh_;

		/*Action library variables*/
		actionlib::SimpleActionServer<script_player::ScriptPlayAction> as_;
		std::string action_name_;
		script_player::ScriptPlayFeedback feedback_;					
		script_player::ScriptPlayResult result_;		

		/* Master Script variables */
		std::string file_paths;   		//Vector of path files for arm, head, video respectively    
		    
		int finish_arm, finish_head, finish_audio, finish_video;
		double cur_time;
		
		DrRobotScriptTeleop drrobotscriptteleop_;
		DrRobotHeadScript drrobotheadscript_;       

		std::string to_string(int value)
		{
			stringstream ss;
			ss << value;
			return ss.str();
		}

    public:
    	boost::thread_group tgroup;
    	boost::timer time;
    	
    //Class constructor
    DrRobotScriptReader(std::string name) : as_(nh_, name, false), action_name_(name)
        {
        //register the goal and feeback callbacks
		as_.registerGoalCallback(boost::bind(&DrRobotScriptReader::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&DrRobotScriptReader::preemptCB, this));
		as_.start();
        	
    	finish_arm = 0;
    	finish_head = 0;
    	finish_audio = 0;
		finish_video = 0;
        	
        }

	void goalCB()
	{
		//Reset helper variables
        	finish_arm = 0;
        	finish_head = 0;
        	finish_audio = 0;
		finish_video = 0;

		//Accept the new goal
		file_paths = as_.acceptNewGoal()->script_folder;
		startUp();

		ROS_INFO("Successfully ran all the scripts");
		result_.success = 1;
		as_.setSucceeded(result_);
			
	}
	void preemptCB()
	{
    	finish_arm = 1;
    	finish_head = 1;
    	finish_audio = 1;
		finish_video = 1;
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
	}


    void readArm()
    {	
    	
    	execute(file_paths + "armtrack.xml", 1);
		finish_arm = 1;
   	}
   	
   	void readHead()
    {	
    	
    	execute(file_paths + "headtrack.xml", 2);
		finish_head = 1;
   	}
   	
   	void readAudio()
    {	
    	
    	execute(file_paths + "audiotrack.xml", 3);
		finish_audio = 1;
   	}
   	
   	void readVideo()
    {	
    	
    	execute(file_paths + "videotrack.xml", 4);
		finish_video = 1;
   	}
       	
    //Read master xml file   	
   	void execute(std::string xmlfile, int option)
   	{
   		xmlDoc *doc;
   	    doc = xmlReadFile(xmlfile.c_str(), NULL, 0);
	    if (doc == NULL){
			printf("error: could not parse file %s\n", xmlfile.c_str());
			exit(-1);
		}

   		xmlNode *root_element_doc = NULL;
        root_element_doc = xmlDocGetRootElement(doc);
    	xmlChar *key_doc;
    	int startTime_doc;
    	std::string xmlFile_doc;

		xmlNode* cur_doc = root_element_doc->xmlChildrenNode;	
		for (cur_doc; cur_doc; cur_doc = cur_doc->next) 
		{
			if (cur_doc->type == XML_ELEMENT_NODE) 
			{
				xmlNode* curchannel_doc = cur_doc->xmlChildrenNode;
				
				for (curchannel_doc; curchannel_doc; curchannel_doc = curchannel_doc->next) 
				{
					if (curchannel_doc->type == XML_ELEMENT_NODE) 
					{
						key_doc = xmlNodeListGetString(doc, curchannel_doc->xmlChildrenNode, 1);
     					if (!xmlStrcmp(curchannel_doc->name, (const xmlChar *)"FileName"))
						{		
							std::string xmlFile_doc_((const char*)key_doc);			
     						xmlFile_doc = file_paths + xmlFile_doc_;
     					}
     					else if (!xmlStrcmp(curchannel_doc->name, (const xmlChar *)"StartTime"))
						{					
     						startTime_doc = atof((const char*)key_doc);
     					}
     					xmlFree(key_doc);      					
	   				}
				}
				//wait for right time to active an action
				while (cur_time < startTime_doc){}
				
				ROS_INFO("File name: %s", xmlFile_doc.c_str());
				ROS_INFO("Time: %d\n", startTime_doc);
				
				//choose right action
				if (option == 1)
				{
					drrobotscriptteleop_.getPose(xmlFile_doc);
				}
				else if (option == 2)
				{
					drrobotheadscript_.getPose(xmlFile_doc);
				}
				else if (option == 3)
				{
					if (access(xmlFile_doc.c_str(), F_OK) == 0) system(("aplay " + xmlFile_doc).c_str());
					else ROS_INFO("File doesn't exist");
				}
				else if (option == 4)
				{
				    if (access(xmlFile_doc.c_str(), F_OK) == 0) system(("mplayer -fs " + xmlFile_doc).c_str());
					else ROS_INFO("File doesn't exist");
				}				
            } 
       	}           	
       	xmlFreeDoc(doc);
		//xmlCleanupParser();
		ROS_INFO("Actions finished");
   	}
   	
   	//Generate a group of threads, each thread will execute a different action 
   	void genThread(const boost::function0<void>& threadfunc)
	{
		tgroup.create_thread( threadfunc );
	}
	
	//A timer to control other actions
	void timer()
	{
		int oldTime = 0;
		time.restart();
		while(finish_arm == 0 || finish_head == 0 || finish_audio == 0 || finish_video == 0)
		{
			cur_time = time.elapsed();
			
			if(cur_time - oldTime >= 1.0)
				ROS_INFO("%f", cur_time);
				
			oldTime = cur_time;
		}
		xmlCleanupParser();
	}
	
	//Start all threads and wait them to finish	
	void startUp()
	{
		genThread(boost::bind( &DrRobotScriptReader::timer, this ));
		genThread(boost::bind( &DrRobotScriptReader::readArm, this ));
		genThread(boost::bind( &DrRobotScriptReader::readHead, this ));
		genThread(boost::bind( &DrRobotScriptReader::readAudio, this ));
		genThread(boost::bind( &DrRobotScriptReader::readVideo, this ));
		tgroup.join_all();
	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "script_player");
	
	DrRobotScriptReader drrobotscriptreader_("script_player");
	ros::spin();	

	return 0;
}
