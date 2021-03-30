//============================================================================
// Name        : drrobot_head_script.cpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Control arms using poses written on a script
//============================================================================

#include "drrobot_head_script.hpp"
    
DrRobotHeadScript::DrRobotHeadScript()
{
  	pub = node.advertise<drrobot_h20_player::HeadCmd>("cmd_pose_head", 1);
  	ros::NodeHandle n_private("~");
        	
   	doc = NULL;
        	
	cmdhead_.neck_x_rotation = NOCONTROL;
	cmdhead_.neck_z_rotation = NOCONTROL;
	cmdhead_.mouth = NOCONTROL;
	cmdhead_.upper_head = NOCONTROL;
	cmdhead_.left_eye = NOCONTROL;
	cmdhead_.right_eye = NOCONTROL;
	cmdhead_.flag = 0;
}

DrRobotHeadScript::~DrRobotHeadScript(){}

//This function is a xml parser
void DrRobotHeadScript::getPose(std::string xmlFile)
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
	//Get child node, one after other getting their contents and set them into a "cmd_pose_head" message
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
   						cmdhead_.neck_x_rotation = 4000 + atoi((const char*)key);
   					}
   					else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo2"))
					{
   						cmdhead_.neck_z_rotation = 3450 + atoi((const char*)key);
  					}
   					else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo3"))
					{						
  						cmdhead_.mouth = 3500 + atoi((const char*)key);       						
   					}
        			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo4"))
					{	
         				cmdhead_.upper_head = 3600 + atoi((const char*)key);  						
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo5"))
					{							
         				cmdhead_.left_eye = 3600 + atoi((const char*)key);        						
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Servo6"))
					{								
         				cmdhead_.right_eye = 3350 + atoi((const char*)key);         						
         			}
         			else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"Time"))
					{
						cmdhead_.flag = atoi((const char*)key);
         			}
         			xmlFree(key);      					
		   		}
			}
			
			ROS_INFO("Publishing...");
			ROS_INFO("Head [%d %d %d %d %d %d]",cmdhead_.neck_x_rotation, cmdhead_.neck_z_rotation, cmdhead_.mouth, cmdhead_.upper_head, cmdhead_.left_eye, cmdhead_.right_eye);
			//Publish "head_pose_cmd" message and waiting for tangy to complete the movement	    
			pub.publish(cmdhead_);			    
			double dur = cmdhead_.flag/1000.0;
			ros::Duration(dur).sleep();
			ROS_INFO("Done!\n"); 
					
		} 
      
	}           	
    xmlFreeDoc(doc);
  	xmlCleanupParser();
}
