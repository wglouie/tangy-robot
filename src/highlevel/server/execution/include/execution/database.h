
/**
 * Author: Tiago Vaquero
 * Date: 2013
 * 
 */

#pragma once //avoid redefining the class when included somewhere else

#include <ros/ros.h>
//#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cstring>
#include <unistd.h>

#include <boost/lexical_cast.hpp>
#include <boost/bimap.hpp> //maps - key -> value
#include <map> //maps - key -> value

//XML libraries
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

//pddl
#include "xmltopddl.h"


using namespace std;

//type definitions
typedef std::map<std::string, std::string> TStrStrMap;
typedef std::pair<std::string, std::string> TStrStrPair;

typedef std::map< std::string, vector<std::string> > TStrVecMap;
typedef std::pair< std::string, vector<std::string> > TStrVecPair;


//TODO:: The maps should be more general. Maybe having just one map to halp all objects and its correspoensing info!!!
// OR we sould not use map at all(?)
class Database
{

public:
    
    // attributes
    
    //Maps (can have the same right side, different from bimap)
	TStrStrMap map_of_locations; // a map of locations (name -> coordinates)
	TStrStrMap stations; // a map of locations (name -> coordinates)
	TStrStrMap users; // a map of sessions (name -> details)
	TStrStrMap sessions; // a map of sessions (name -> details)
	TStrStrMap games; // a map of sessions (name -> details)
	
	//TStrVecMap objects; // a map of existing objects (type -> Objects)

    string path;

    
    
    /**
     * Constructors
     **/
    Database(){
    }
    
    Database(string folderpath){
    	path = folderpath;    	
        getData();
    }       

    ~Database(){
    }
    
    
    /**
	 * This function reads and sets the available locations, users and sessions
	 */
	void getData(){
		
		//get static data
		getMapOfLocations();
		getChargingStations();
		getUsers();		
		//get dynamic data
		getSessions();
		getGames();
		
	}
    
   
	
    
    /**
     * This function reads and sets the available locations
     */
    void getMapOfLocations(){
    	
    	//getObjectsPDDLAttributes("locations.xml","location",&map_of_locations);
    	getObjectsAttribute("locations.xml","location",&map_of_locations);
		//map_of_locations.insert(TStrStrPair("l0","0.550,0.000,0.000,0.000,0.000,-0.707,0.707"));
		//map_of_locations.insert(TStrStrPair("l1","3.786,1.590,0.000,0.000,0.000,-0.028,1.000"));
		//map_of_locations.insert(TStrStrPair("l2","4.456,3.811,0.000,0.000,0.000,-0.006,1.000"));     	
		//print the map		
		printTStrStrMap(&map_of_locations);
		
    }
      
    
    /**
     * This function reads and sets the available charging stations
     */
    void getChargingStations(){
    	
    	//getObjectsPDDLAttributes("locations.xml","location",&map_of_locations);
    	getObjectsAttribute("stations.xml","station",&stations);  	
		//print the map		
		printTStrStrMap(&stations);
		
    }

    /**
	 * This function reads and sets the available users
	 */
	void getUsers(){
		
		getObjectsAttribute("users.xml","user",&users);
		//getObjectsPDDLAttributes("users.xml","user",&users);
		//users.insert(TStrStrPair("user1","Rafael"));
		//users.insert(TStrStrPair("user2","Matthew"));
		//users.insert(TStrStrPair("user3","Tiago"));		
		//print the map		
		printTStrStrMap(&users);			
					
	}
	    
    
    /**
	 * This function reads and sets the available sessions
	 */
	void getSessions(){
		
		//getObjectsPDDLAttributes("sessions.xml","session",&sessions);
		getObjectsAttribute("sessions.xml","session",&sessions);		
		//print the map		
		printTStrStrMap(&sessions);		
	}

	
    /**
	 * This function reads and sets the available games
	 */
	void getGames(){
		
		//getObjectsPDDLAttributes("games.xml","bingo",&games);
		getObjectsAttribute("games.xml","bingo",&games);		
		//print the map		
		printTStrStrMap(&games);		
	}
	
	
	
	
	
	//PDDL
	
	/**
		 * This function reads and sets the global variables,
		 *  and generates the PDDL content (init)
		 */
		string getCurrentGlobalVariablesInPDDL(){

			string pddlsection = "";	
			pddlsection = getObjectsPDDLAttributes("globals.xml","global",NULL);
			/*
			pddlsection += "      (= (distance l0 l1) 20) \n"; 
			pddlsection += "      (= (distance l1 l0) 20) \n"; 
			pddlsection += "      (= (distance l1 l2) 20) \n"; 
			pddlsection += "      (= (distance l2 l1) 20) \n"; 
			pddlsection += "      (= (distance l2 l3) 20) \n"; 
			pddlsection += "      (= (distance l3 l2) 20) \n"; 
			pddlsection += "      (= (distance l0 l2) 20) \n"; 
			pddlsection += "      (= (distance l2 l0) 20) \n";
			pddlsection += "      \n"; 
			
			pddlsection += "      (= (distancetostation l0 cs1) 0) \n"; 
			pddlsection += "      (= (distancetostation l1 cs1) 20) \n"; 
			pddlsection += "      (= (distancetostation l2 cs1) 60) \n"; 
			pddlsection += "      (= (distancetostation l3 cs1) 40) \n"; 
			pddlinit += "      \n"; 
			*/	
			//printf("============\n");
			//printf("%s", pddlsection.c_str());
			//printf("============\n");
			return pddlsection;
			
		}
	
	
	/**
	 * This function reads and sets the available charging stations,
	 *  and generates the PDDL content (init) for the stations
	 */
	string getCurrentLocationsInPDDL(){

		string pddlsection = "";		
		pddlsection = getObjectsPDDLAttributes("locations.xml","location",&map_of_locations);
		/*
		pddlsection += "      (connected l0 l1) \n";        
		pddlsection += "      (connected l1 l0) \n";
		pddlsection += "      (connected l1 l2) \n";
		pddlsection += "      (connected l2 l1) \n";
		pddlsection += "      (connected l2 l3) \n";
		pddlsection += "      (connected l3 l2) \n";
		*/	
		//printf("============\n");
		//printf("%s", pddlsection.c_str());
		//printf("============\n");
		return pddlsection;
		
	} 
	
    /**
 	 * This function reads and sets the available charging stations,
 	 *  and generates the PDDL content (init) for the stations
 	 */
 	string getCurrentStationsInPDDL(){

 		string pddlsection = "";		
 		pddlsection = getObjectsPDDLAttributes("stations.xml","station",&stations);
 		//pddlsection += "      (idle cs1) \n"; 
 		//pddlsection += "      (availableat cs1 l0) \n";  		
 		//printf("============\n");
 		//printf("%s", pddlsection.c_str());
 		//printf("============\n");
 		return pddlsection;
 		
 	} 
	
	
    /**
	 * This function reads and sets the available sessions,
	 *  and generates the PDDL content (init) for the sessions
	 */
	string getCurrentUsersInPDDL(){
		
		string pddlsection = "";		
		pddlsection = getObjectsPDDLAttributes("users.xml","user",&users);
		/*
		//user1
		pddlsection += "      (at user1 l1) \n";
		pddlsection += "      (available user1) \n";
		pddlsection += "      (notinteracting user1) \n";              
		//user2
		pddlsection += "      (at user2 l2) \n";
		pddlsection += "      (available user2) \n";
		pddlsection += "      (notinteracting user2) \n";       
		//user3
		pddlsection += "      (at user3 l3) \n";
		pddlsection += "      (available user3) \n";
		pddlsection += "      (notinteracting user3) \n";
		pddlsection += "      \n";
		*/    
		//printf("============\n");
		//printf("%s", pddlsection.c_str());
		//printf("============\n");
		return pddlsection;
		
	}
	

    /**
	 * This function reads and sets the available sessions,
	 *  and generates the PDDL content (init) for the sessions
	 */
	string getCurrentSessionsInPDDL(){

		string pddlsection = "";		
		pddlsection = getObjectsPDDLAttributes("sessions.xml","session",&sessions);
		//printf("============\n");
		//printf("%s", pddlsection.c_str());
		//printf("============\n");
		return pddlsection;
		
	}
	
    /**
	 * This function reads and sets the available sessions,
	 *  and generates the PDDL content (init) for the sessions
	 */
	string getCurrentGamesInPDDL(){

		string pddlsection = "";		
		pddlsection = getObjectsPDDLAttributes("games.xml","bingo",&games);
		//printf("============\n");
		//printf("%s", pddlsection.c_str());
		//printf("============\n");
		return pddlsection;
		
	}
	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	/**
	 * This function returns an object's info to be used as extra info in the command info that will be sent to the robots 
	 */
	string getObjectInfo(string obj, string type){
		//set info
		//printf("Object [%s], type [%s]\n", obj.c_str(), type.c_str());		
		
		string info = "";
		if (type == "location")
			info = getInfo(obj,&map_of_locations);
		else if (type == "user")
			info = getInfo(obj,&users);
		else if (type == "session")
			info = getInfo(obj,&sessions);
		else if (type == "game")
			info = getInfo(obj,&games);
			
		return info;
	}
	
	
	 /**
	 * This function returns an object's info to be used as extra info in the command info that will be sent to the robots 
	 */
	string getInfo(string obj, TStrStrMap *map){
		
		string info = "";
		
		//set info
		//printf("Object: [%s]\n", obj.c_str());
				
        //print the map		
		//printTStrStrMap(map);	
				
		//find if the obj exists
		TStrStrMap::iterator p = map->find(obj.c_str());

		if (!obj.empty() && p != map->end()){
			//printf("Origin location: %s\n",act.objects.at(0).c_str());
			string target_info = p->second;
			if (!target_info.empty() ){
				info = obj + ":" +target_info;
				info = target_info;
			}
		}
		
		return info;
	}
		
	
	/**
	 * This function reads and sets the available objects and their atttributes,
	 *  and generates the PDDL content (init) for the sessions
	 */
	string getObjectsPDDLAttributes(string sourcefile, string objtype, TStrStrMap *map){
		
		string pddlsection = "";
		
		if (map != NULL)
			map->clear();
				
		
		//Read a given xml file
		string file = path + "/resources/"+sourcefile;
		//printf("FILE: %s\n",locationsfile.c_str());
		
		xmlDoc *doc = xmlReadFile(file.c_str(), NULL, 0);
		if (doc == NULL){
			printf("ERROR: could not parse file %s\n", file.c_str());
			//exit(-1);
			return "";
		}		
		
		//Point to root node
		xmlNodePtr root_element = NULL;
		root_element = xmlDocGetRootElement(doc);
		
		xmlChar *key;
		//Get child node, one after other getting their contents and set them into a "cmd_pose_head" message
			
		xmlNodePtr cur= root_element->xmlChildrenNode;		
		
		//printf("READING THE XML FILE - SESSIONS\n");
		
		//main node attributes
		xmlChar *id;
		xmlChar *value;
		xmlAttr *pddl_marker;

		vector<string> list; //list of non-pddl values
		
		for (cur; cur; cur = cur->next)
		{
			if (cur->type == XML_ELEMENT_NODE)
			{
				if ((!xmlStrcmp(cur->name, (const xmlChar *)objtype.c_str()))) {
										
					//Get id

					//id
					id = xmlGetProp(cur, (const xmlChar *)"id");
					string idstr = (char *) id;
					//printf("%s:%s\n",objtype.c_str(),idstr.c_str());				
					xmlFree(id);
					
					//variables for non-pddl attributes
					string data = "";
					list.clear();
					
					
					//PDDL
					string element_name = (char *) cur->name;
					pddlsection += "      ;;" + element_name + ": "+idstr+" \n";					
					xmlNodePtr cur_sub = cur->xmlChildrenNode;
					//Go through the xml nodes as attributes/predicates/fluent of the object 
					while (cur_sub != NULL) {
						if (cur_sub->type == XML_ELEMENT_NODE && (!xmlStrcmp(cur_sub->name, (const xmlChar *)"attribute"))){
							
							//Get PDDL attributes
							bool ispddl = false;
							//Check if the attribute is meant to be PDDL attribute
							pddl_marker = xmlHasProp(cur_sub, (const xmlChar *)"pddl");
							if (pddl_marker != NULL){
								//Check the value of the attribute. It must be true to add it to the pddl
								string pddl_markerstr = (char *) pddl_marker->children->content; //get the value of the xml attribute
								if (pddl_markerstr == "true"){
									pddlsection += xmlAttributeToPDDL(idstr,cur_sub);
									ispddl = true;
								}
							}
							
							//Get non-PDDL attributes
							//if it is not pddl them add it to the map to be sent together with the commands (info to the agents/robots)
							if(!ispddl){									
								//value
								value = xmlGetProp(cur_sub, (const xmlChar *)"value");
								string valuestr = (char *) value;
								//printf(" - value: %s\n", valuestr.c_str());					
								xmlFree(value);
								list.push_back(valuestr);
							}
						}

						cur_sub = cur_sub->next;
					}						
					pddlsection += "      \n";
					
					
					//Create non-pddl data
					for(int i=0;i<list.size();++i){						
						if (i == list.size()-1)//last element
							data += list.at(i);
						else
							data += list.at(i) + ",";					
					}
					

					//add data to the map
					if (map != NULL)
						map->insert(TStrStrPair(idstr,data));														
																	
					//pddlsection += "      \n";
				}				
			}
		}
		//printf("END OF READING THE XML FILE - SESSIONS\n");
			
		//printf("============\n");
		//printf("%s", pddlsection.c_str());
		//printf("============\n");
		
		return pddlsection;
		
	}
	
	
	/**
	 * This function reads and sets the available object and their attributes in a given map,
	 * 
	 */
	void getObjectsAttribute(string sourcefile, string objtype, TStrStrMap *map){
					
		map->clear();
				
		
		//Read a given xml file
		string file = path + "/resources/"+sourcefile;
		//printf("FILE: %s\n",locationsfile.c_str());
		
		xmlDoc *doc = xmlReadFile(file.c_str(), NULL, 0);
		if (doc == NULL){
			printf("ERROR: could not parse file %s\n", file.c_str());
		}		
		
		//Point to root node
		xmlNodePtr root_element = NULL;
		root_element = xmlDocGetRootElement(doc);
		
		xmlChar *key;
		//Get child node, one after other getting their contents and set them into a "cmd_pose_head" message
			
		xmlNodePtr cur= root_element->xmlChildrenNode;		
		
		//printf("READING THE XML FILE - SESSIONS\n");
		
		//main node attributes
		xmlChar *id;
		xmlChar *value;
		xmlAttr *pddl_marker;

		vector<string> list; //list of non-pddl values
		
		for (cur; cur; cur = cur->next)
		{
			if (cur->type == XML_ELEMENT_NODE)
			{
				if ((!xmlStrcmp(cur->name, (const xmlChar *)objtype.c_str()))) {
										
					//Get id

					//id
					id = xmlGetProp(cur, (const xmlChar *)"id");
					string idstr = (char *) id;
					//printf("%s:%s\n",objtype.c_str(),idstr.c_str());				
					xmlFree(id);
					
					//variables for non-pddl attributes
					string data = "";
					list.clear();
					
					
					//PDDL
					string element_name = (char *) cur->name;		
					xmlNodePtr cur_sub = cur->xmlChildrenNode;
					//Go through the xml nodes as attributes/predicates/fluent of the object 
					while (cur_sub != NULL) {
						if (cur_sub->type == XML_ELEMENT_NODE && (!xmlStrcmp(cur_sub->name, (const xmlChar *)"attribute"))){
							
							//Get PDDL attributes
							bool ispddl = false;
							//Check if the attribute is meant to be PDDL attribute
							pddl_marker = xmlHasProp(cur_sub, (const xmlChar *)"pddl");
							if (pddl_marker != NULL){
								//Check the value of the attribute. It must be true to add it to the pddl
								string pddl_markerstr = (char *) pddl_marker->children->content; //get the value of the xml attribute
								if (pddl_markerstr == "true"){
									ispddl = true;
								}
							}
							
							//Get non-PDDL attributes
							//if it is not pddl them add it to the map to be sent together with the commands (info to the agents/robots)
							if(!ispddl){									
								//value
								value = xmlGetProp(cur_sub, (const xmlChar *)"value");
								string valuestr = (char *) value;
								//printf(" - value: %s\n", valuestr.c_str());					
								xmlFree(value);
								list.push_back(valuestr);
							}
						}

						cur_sub = cur_sub->next;
					}											
					
					//Create non-pddl data
					for(int i=0;i<list.size();++i){						
						if (i == list.size()-1)//last element
							data += list.at(i);
						else
							data += list.at(i) + ",";					
					}
					
					//add data to the map
					map->insert(TStrStrPair(idstr,data));														
																	
				}				
			}
		}
		//printf("END OF READING THE XML FILE - SESSIONS\n");
	}
	
	
	
	string getCurrentObjectsInPDDL(){
		string pddlsection = "";
		//pddlsection = getPDDLObjects("objects.xml",&objects);
		pddlsection = getPDDLObjects("objects.xml");		
		
        //pddlsection += "      l0 l1 l2 l3 - Location \n"; //locations
        //pddlsection += "      r1 - Robot \n"; //robots
        //pddlsection += "      user1 user2 usesr3 - User \n"; //users  
        //pddlsection += "      s1 s2 s3 - TelepresenceSession \n"; //telepresence sessions  
        //pddlsection += "      cs1 - ChargingStation \n"; //charging station		
		return pddlsection;
		
	}
	
	/**
	 * This function reads and sets the available objects ,
	 *  and generates the PDDL content (init) for the objects
	 */
	//string getPDDLObjects(string sourcefile, TStrVecMap *map){
	string getPDDLObjects(string sourcefile){
		
		string pddlsection = "";
		
		//map->clear();
				
		
		//Read a given xml file
		string file = path + "/resources/"+sourcefile;
		//printf("FILE: %s\n",locationsfile.c_str());
		
		xmlDoc *doc = xmlReadFile(file.c_str(), NULL, 0);
		if (doc == NULL){
			printf("ERROR: could not parse file %s\n", file.c_str());
			//exit(-1);
			return "";
		}		
		
		//Point to root node
		xmlNodePtr root_element = NULL;
		root_element = xmlDocGetRootElement(doc);
		
		xmlChar *key;
		//Get child node, one after other getting their contents and set them into a "cmd_pose_head" message
			
		xmlNodePtr cur= root_element->xmlChildrenNode;		
		
		//printf("READING THE XML FILE - SESSIONS\n");

		for (cur; cur; cur = cur->next)
		{
			if (cur->type == XML_ELEMENT_NODE)
			{
				pddlsection += xmlObjectstoPDDL(cur);
			}
		}
		//printf("END OF READING THE XML FILE - SESSIONS\n");
			
		//printf("============\n");
		//printf("%s", pddlsection.c_str());
		//printf("============\n");
		
		return pddlsection;
		
	}
	
	

	
	/**
     * this function checks in which discrete location (region) of the map the robot is in
     */
    string coordinatesToLocation(double x_position, double y_position){

    	string location = "";
    	//TODO: The regions must come from a another source (e.g., an XML file). We cannot hardcode it here
    	//Location 0
		if(x_position >= -1.400 && x_position <= 3.250 && y_position >= -2.200 && y_position <= 1.900) {
			//printf("\nl0\n");
			location = "l0";
		}
		//Location 1
		else if(x_position > 3.250 && x_position <= 7.100 && y_position >= -2.200 && y_position <= 2.700 ){
			//printf("\nl1\n");
			location = "l1";
		}
		//Location 2
		else if(x_position > 3.250 && x_position <= 7.100 && y_position > 2.700 && y_position <= 5.400){
			//printf("\nl2\n");
			location = "l2";
		}
		//Location 3
		else if(x_position > 3.250 && x_position <= 7.100 && y_position > 5.400 && y_position <= 8.200){
				//printf("\nl3\n");
			location = "l3";
		}
		//Unknown location
		else
			printf("\nThe robot is lost... LOL\n");
			
		printf("  - location: %s.\n", location.c_str());
		
		return 	location;
	}
    
    /**
     * This function prints a TStrStrMap
     */
    void printTStrStrMap(TStrStrMap *map){
    	TStrStrMap::iterator p;		
		for(p = map->begin(); p!=map->end(); ++p)
		{
			std::cout << p->first << " -----> " << p->second << std::endl;		
		}
    }
    

    

};

