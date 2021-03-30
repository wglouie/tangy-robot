#include <help_indicators/help_indicators.h>
#include <boost/lexical_cast.hpp>

// check if new_help_lghts belong in help_indicators
void Help::update_help_indicators(pcl::PointCloud<pcl::PointXYZ> cloud_filtered) {

	std::vector<help_indicators::triangle> new_help_indicators_copy = new_help_indicators;

	// convert new_help_indicators into real world x,y,z (cm) and base_link frame
	for(int i = 0; i < new_help_indicators_copy.size(); i++) {
		help_indicators::triangle *new_help_indicator = &new_help_indicators_copy.at(i);		
		double x = cloud_filtered.at(new_help_indicator->x,new_help_indicator->y).x;
		double y = cloud_filtered.at(new_help_indicator->x,new_help_indicator->y).y;
		double z = cloud_filtered.at(new_help_indicator->x,new_help_indicator->y).z;
        new_help_indicator->image_x = new_help_indicator->x;
        new_help_indicator->image_y = new_help_indicator->y;
		new_help_indicator->x = x;
		new_help_indicator->y = y;
		new_help_indicator->z = z;
		// ROS_INFO("The numbers are %f, %f, %f", x, y, z);
	}

	// go through all new_help_indicators
	for(int i =0; i < new_help_indicators_copy.size(); i++) {
		help_indicators::triangle new_help_indicator = new_help_indicators_copy.at(i);
		// filter out triangles too close or too far
		if(new_help_indicator.z < kinect_min_dist || new_help_indicator.z > kinect_max_dist) {
			#ifdef DEBUG_HELP_INDICATORS
			ROS_ERROR("Help Indicator Filtered");
			#endif
			continue;
		}

		// if the new_help_indicators has been seen 15 or more times its a potential candidate but we still need to check if its already added 
		#ifdef DEBUG_HELP_INDICATORS
		ROS_ERROR("Seen triangle %d, %d times", i, times_seen(new_help_indicator));
		#endif		
		if(times_seen(new_help_indicator) >= 10) {
			#ifdef DEBUG_HELP_INDICATORS
			ROS_ERROR("New Help Indicator Detected");
			#endif
			bool included = false;
			for(int j = 0; j < help_indicators.size(); j++) {
				help_indicators::triangle help_indicator = help_indicators.at(j);
				double dist = get_distance(help_indicator.x, help_indicator.y,new_help_indicator.x,new_help_indicator.y);
				//if the new_help_indicators is within 40cm of another help_indicators we consider it to be the same help_indicators
				if(dist < 0.40)
					included = true;
					#ifdef DEBUG_HELP_INDICATORS
					ROS_ERROR("New Help Indicator Already Added");
					#endif
			}
			// if the new_help_indicators is not withing 40cm of any help_indicators then we add it to the list
			if(!included) {
				// check that they're not nan's (nan =/= nan)
				if(new_help_indicator.x == new_help_indicator.x && new_help_indicator.y == new_help_indicator.y && new_help_indicator.z == new_help_indicator.z) {
					ROS_INFO("Publishing new help indicator to help transformer");
					help_indicators.push_back(new_help_indicator);
					// transform to base_link coordinates
					pub_transformer.publish(new_help_indicator);

					// search through new_help_indicators until we find the one with the specified name to be deleted
					for(int i = 0; i < new_help_indicators.size(); i++) {
						if(new_help_indicators.at(i).name == new_help_indicator.name) {
						    new_help_indicators.erase(new_help_indicators.begin() + i);
							break;
						}
					}

				}
				else {
					#ifdef DEBUG_HELP_INDICATORS
					ROS_ERROR("Can't get depth because of NAN's");
					#endif
				}
			}
		} else {
			//ROS_INFO("Not seen enough times");
		}
	}
}


