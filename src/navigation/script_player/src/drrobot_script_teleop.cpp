//============================================================================
// Name        : drrobot_script_teleop.cpp
// Author      : Ananias Paiva
// Copyright   : 
// Description : Control arms using poses written on a script
//============================================================================

#include "drrobot_script_teleop.hpp"
    
DrRobotScriptTeleop::DrRobotScriptTeleop()
{
  	pub = node.advertise<drrobot_h20_arm_player::ArmCmd>("cmd_arm", 1);
  	ros::NodeHandle n_private("~");
        	
   	doc = NULL;
        	
   	cmdarm_.right_arm[0] = NOCONTROL;
   	cmdarm_.right_arm[1] = NOCONTROL;
   	cmdarm_.right_arm[2] = NOCONTROL;
  	cmdarm_.right_arm[3] = NOCONTROL;
   	cmdarm_.right_arm[4] = NOCONTROL;
   	cmdarm_.right_arm[5] = NOCONTROL;
    cmdarm_.right_arm[6] = NOCONTROL;
   	cmdarm_.right_arm[7] = NOCONTROL;
   	cmdarm_.left_arm[0] = NOCONTROL;
   	cmdarm_.left_arm[1] = NOCONTROL;
   	cmdarm_.left_arm[2] = NOCONTROL;
   	cmdarm_.left_arm[3] = NOCONTROL;
   	cmdarm_.left_arm[4] = NOCONTROL;
   	cmdarm_.left_arm[5] = NOCONTROL;
   	cmdarm_.left_arm[6] = NOCONTROL;
   	cmdarm_.left_arm[7] = NOCONTROL;
   	cmdarm_.vel = 1000;
   	cmdarm_.speed = false;
}

DrRobotScriptTeleop::~DrRobotScriptTeleop(){}

//This function is a xml parser
void DrRobotScriptTeleop::getPose(std::string xmlFile)
{	
        	
 	sleep(1);
    
    //Read a given xml file    	
    doc = xmlReadFile(xmlFile.c_str(), NULL, 0);
    if (doc == NULL){
		printf("error: could not parse file %s\n", xmlFile.c_str());
		exit(-1);
	}
	//Point to root node	   
    xmlNode *root_element = NULL;
    root_element = xmlDocGetRootElement(doc);
   	xmlChar *key;
	//Get child node, one after other getting their contents and set them into a "cmd_arm" message
	xmlNode* cur = root_element->xmlChildrenNode;	
	for (cur; cur; cur = cur->next) 
	{
		if (cur->type == XML_ELEMENT_NODE) 
		{
			xmlNode* curchannel = cur->xmlChildrenNode;
				
			for (curchannel; curchannel; curchannel = curchannel->next) 
			{
				if (curchannel->type == XML_ELEMENT_NODE) 
				{
					key = xmlNodeListGetString(doc, curchannel->xmlChildrenNode, 1);
					if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo1"))
					{
   						cmdarm_.right_arm[0] = 1700 + atoi((const char*)key);
   					}
   					else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo2"))
					{
   						cmdarm_.right_arm[1] = 1269 + atoi((const char*)key);
  					}
   					else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo3"))
					{						
  						cmdarm_.right_arm[2] = 1625 + atoi((const char*)key);       						
   					}
        			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo4"))
					{	
         				cmdarm_.right_arm[3] = 1545 + atoi((const char*)key);  						
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo5"))
					{							
         				cmdarm_.right_arm[4] = 1710 + atoi((const char*)key);        						
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo6"))
					{								
         				cmdarm_.right_arm[5] = 2000 + atoi((const char*)key);         						
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo7"))
					{
						cmdarm_.right_arm[6] = 2300 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo8"))
					{
						cmdarm_.right_arm[7] = 760 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo9"))
					{
						cmdarm_.left_arm[0] = 1290 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo10"))
					{
						cmdarm_.left_arm[1] = 1720 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo11"))
					{
						cmdarm_.left_arm[2] = 1570 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo12"))
					{
						cmdarm_.left_arm[3] = 1520 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo13"))
					{
						cmdarm_.left_arm[4] = 1940 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo14"))
					{
						cmdarm_.left_arm[5] = 2010 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo15"))
					{
						cmdarm_.left_arm[6] = 1240 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo16"))
					{
						cmdarm_.left_arm[7] = 2200 + atoi((const char*)key);
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Time"))
					{
						cmdarm_.vel = atoi((const char*)key);
         			}
         			xmlFree(key);      					
		   		}
			}
			
			ROS_INFO("Publishing...");
			ROS_INFO("right_arm [%d %d %d %d %d %d %d %d]",cmdarm_.right_arm[0], cmdarm_.right_arm[1], cmdarm_.right_arm[2], cmdarm_.right_arm[3], cmdarm_.right_arm[4], cmdarm_.right_arm[5], cmdarm_.right_arm[6], cmdarm_.right_arm[7]);
			ROS_INFO("left_arm [%d %d %d %d %d %d %d %d]",cmdarm_.left_arm[0], cmdarm_.left_arm[1], cmdarm_.left_arm[2], cmdarm_.left_arm[3], cmdarm_.left_arm[4], cmdarm_.left_arm[5], cmdarm_.left_arm[6], cmdarm_.left_arm[7]);
			ROS_INFO("vel %d", cmdarm_.vel);
			if (cmdarm_.speed == 0)	ROS_INFO("speed 'FALSE'");
			else ROS_INFO("speed 'TRUE'\n"); 
			
			//Publish "arm_cmd" message and waiting for tangy to complete the movement	    
			pub.publish(cmdarm_);
			double dur = cmdarm_.vel/1000.0;
			ros::Duration(dur).sleep();			    
			ROS_INFO("Done!\n"); 
					
		} 
      
	}           	
    xmlFreeDoc(doc);
  	xmlCleanupParser();
}
