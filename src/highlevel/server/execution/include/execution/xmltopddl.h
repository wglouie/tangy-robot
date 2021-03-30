
/**
 * Author: Tiago Vaquero
 * Date: 2013
 * 
 * 
 * This library provides funtions for translating xml data into pddl
 *  
 */

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

//XML libraries
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include <libxml/tree.h>



using namespace std;


/**
 * This function traslates an xml attribute node into a pddl fact
 */
string xmlAttributeToPDDL(string objstr, xmlNodePtr cur){
	
	string pddlline = "";
	
	//atributes
	xmlChar *attrname;
	xmlChar *value;
	xmlChar *type;
	xmlAttr *time_stamp;
	xmlAttr *pddl_marker;
	
	if (cur->type == XML_ELEMENT_NODE && (!xmlStrcmp(cur->name, (const xmlChar *)"attribute"))) {	
		
		//key = xmlNodeListGetString((cur_sub, cur->xmlChildrenNode, 1);
		//printf("keyword: %s\n", key);
		//xmlFree(key);
		
		//Check if the attribute is meant to be PDDL attribute
		pddl_marker = xmlHasProp(cur, (const xmlChar *)"pddl");
		if (pddl_marker != NULL){
			//Check the value of the attribute. It must be true to add it to the pddl
			string pddl_markerstr = (char *) pddl_marker->children->content; //get the value of the xml attribute
			if (pddl_markerstr == "true"){
				//BASICS
				//attribute name
				attrname = xmlGetProp(cur, (const xmlChar *)"name");
				string attrnamestr = (char *) attrname;
				//printf(" - name: %s\n", attrnamestr.c_str());					
				xmlFree(attrname);
				//type
				type = xmlGetProp(cur, (const xmlChar *)"type");
				string typestr = (char *) type;
				//printf(" - type: %s\n", typestr.c_str());					
				xmlFree(type);
				//value
				value = xmlGetProp(cur, (const xmlChar *)"value");
				string valuestr = (char *) value;
				//printf(" - value: %s\n", valuestr.c_str());					
				xmlFree(value);
																		
				//ATTRIBUTE'S PARAMETERS: each attribute might have more parameters
				vector<string> paramlist; //list of non-pddl values
				xmlNodePtr cur_sub = cur->xmlChildrenNode;
				while (cur_sub != NULL) {
					if (cur_sub->type == XML_ELEMENT_NODE && (!xmlStrcmp(cur_sub->name, (const xmlChar *)"parameter"))){						
						//type
						type = xmlGetProp(cur_sub, (const xmlChar *)"type");
						string typestr = (char *) type;				
						xmlFree(type);
						//value
						value = xmlGetProp(cur_sub, (const xmlChar *)"value");
						string valuestr = (char *) value;				
						xmlFree(value);
						//set list
						paramlist.push_back(valuestr);						
					}
					cur_sub = cur_sub->next;
				}
				string paramsstr = "";
				for(int i=0;i<paramlist.size();++i){						
					if (i == paramlist.size()-1)//last element
						paramsstr += paramlist.at(i);
					else
						paramsstr += paramlist.at(i) + " ";					
				}				
				
				
				//BASIC FLUENT CONTENT
				string fluent_content = attrnamestr;
				if (objstr != "global") //if it is global variable do not put the name of the object
					fluent_content += " " + objstr;
				if (paramlist.size() > 0)
					fluent_content += " " + paramsstr;
				
				//CHECK the TYPE of the value
				if (typestr == "bool"){
					pddlline =  "(" + fluent_content + ")";
					if (valuestr == "false"){
						pddlline =  "(not " + pddlline + ")";
					}								
				}
				else if (typestr == "int" || typestr == "float"){
					pddlline = "(= (" + fluent_content + ") " + valuestr + ")";
				}
				else{
					pddlline =  "(" + fluent_content + " " + valuestr + ")";
				}
				
				//TIME STAMP
				time_stamp = xmlHasProp(cur, (const xmlChar *)"time_stamp");
				if (time_stamp != NULL){
					string time_stampstr = (char *) time_stamp->children->content; //get the value of the xml attribute
					//printf(" - value: %s\n", time_stampstr.c_str());					
					xmlFree(time_stamp);
					pddlline = "(at " + time_stampstr + " " + pddlline + ")";
				}
				
				pddlline = "      "+pddlline+"\n";					
				
			}
	
		}
		
	}
	
	return pddlline;
	
}



/**
 * This function receives a xml node/object and put translate it as PDDL fluents 
 */
string xmlObjectAttributesToPDDL(xmlNodePtr cur){
	
	string pddlsection = "";
	
	xmlChar *id;
	xmlChar *value;
	xmlAttr *pddl_marker;
	
	if (cur->type == XML_ELEMENT_NODE)
	{								
			//Get id

			//id
			id = xmlGetProp(cur, (const xmlChar *)"id");
			string idstr = (char *) id;
			//printf("%s:%s\n",objtype.c_str(),idstr.c_str());				
			xmlFree(id);
						
			
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
				}

				cur_sub = cur_sub->next;
			}						
			pddlsection += "      \n";			
	}
	
	return pddlsection;
	
}


/**
 * This function translates a class node with objects as children into a PDDL :object section
 */
string xmlObjectstoPDDL(xmlNodePtr cur){
	
	string pddlsection = "";
	
	//main node attributes
	xmlChar *type_id;
	xmlChar *id;
	xmlChar *value;	
	
	if ((!xmlStrcmp(cur->name, (const xmlChar *)"class"))) {
											
		//type id
		type_id = xmlGetProp(cur, (const xmlChar *)"id");
		string type_idstr = (char *) type_id;
		//printf("%s:%s\n",objtype.c_str(),idstr.c_str());				
		xmlFree(type_id);
		
		vector<string> list; //list of non-pddl values
		
		
		//PDDL
		string element_name = (char *) cur->name;					
		xmlNodePtr cur_sub = cur->xmlChildrenNode;
		//Go through the xml nodes as attributes/predicates/fluent of the object 
		while (cur_sub != NULL) {
			if (cur_sub->type == XML_ELEMENT_NODE && (!xmlStrcmp(cur_sub->name, (const xmlChar *)"object"))){
				
				//type id
				id = xmlGetProp(cur_sub, (const xmlChar *)"id");
				string idstr = (char *) id;
				//printf("%s:%s\n",objtype.c_str(),idstr.c_str());				
				xmlFree(id);
				list.push_back(idstr);
			}

			cur_sub = cur_sub->next;
		}	
		
		string data = "";
		//Create non-pddl data
		for(int i=0;i<list.size();++i){						
			data += list.at(i) + " ";					
		}
		if (list.size() > 0)
			pddlsection += "      "+ data + "- " + type_idstr + "\n";
		

		//add data to the map
		//map->insert(TStrVecPair(type_idstr,list));														
														
	}
	
	return pddlsection;
	
}





