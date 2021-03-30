#include <help_indicators/help_indicators.h>

void Help::remove_duplicates(std::vector<help_indicators::triangle> &new_triangles) {
    // temporarly store the unduplicated triangles
    std::vector<help_indicators::triangle> temp;
    // if any triangles further down the vector are near we consider it a duplicate
    for(int i = 0; i < new_triangles.size(); i++) {
        help_indicators::triangle  safe_triangle = new_triangles.at(i);
        bool unduplicated = true;
        for(int j = i+1; j < new_triangles.size(); j++) {
            help_indicators::triangle suspect_triangle = new_triangles.at(j);
            double dist = get_distance(safe_triangle.x, safe_triangle.y, suspect_triangle.x, suspect_triangle.y);  
            // use double the checking distance because we don't want to count 1 triangle twice later
            if(dist < 20) {
                unduplicated = false;
            }
        }
        if(unduplicated == true) {
            temp.push_back(new_triangles.at(i));
        } 
    }
    new_triangles = temp; 
}

// check how to update new_help_indicators and what to do with the triangles currently being seen by the kinect 
void Help::update_new_help_indicators(std::vector<help_indicators::triangle> new_triangles) {
    remove_duplicates(new_triangles);

	// initialize the remaining new_triangles as having not been added to new_help_indicators
	for(int i = 0; i < new_triangles.size(); i++) {
    		new_triangles.at(i).added = false;
	}

    // check if triangle has been seen recently and check if new_triangles needed to be added
    int size = new_help_indicators.size();
    for(int i = 0; i < new_help_indicators.size(); i++) {
       help_indicators::triangle *new_help_indicator = &new_help_indicators.at(i);
        bool seen = false;
        for(int j = 0; j < new_triangles.size(); j++) {
            help_indicators::triangle *new_triangle = &new_triangles.at(j); 
            double dist = get_distance(new_help_indicator->x, new_help_indicator->y, new_triangle->x, new_triangle->y);
            // anything new_triangle a radius of 10pixels from the new_help_indicator is counted as the same triangle      
            if(dist < 10) {
                seen = true;
                new_triangle->added = true;
            } 
        }
	// sliding window of times seen
        for(int j = 0; j < 19; j++) {
            new_help_indicator->seen[j] = new_help_indicator->seen[j+1];
        } 
        if(seen) {
		#ifdef DEBUG_HELP_INDICATORS
		ROS_ERROR("SEEN");
		#endif
            new_help_indicator->seen[19] = true;
        } else {//(/*times_seen(*new_help_indicator) < 15*/){
		#ifdef DEBUG_HELP_INDICATORS
		ROS_ERROR("NOT SEEN");
		#endif
            new_help_indicator->seen[19] = false;
			// if we haven't seen it in the passed 20 frames remove it from the list
            if(times_seen(*new_help_indicator) == 0) {
                new_help_indicators.erase(new_help_indicators.begin() + i);
            } 
        }       
    }

	// for any of the  new_triangles that were not in new_help_indicators, add them
    for(int j = 0; j < new_triangles.size(); j++) {    
        help_indicators::triangle new_triangle = new_triangles.at(j);
        if(new_triangle.added == false) {
			count++;
			new_triangle.name = name + boost::lexical_cast<std::string>(count);
            new_help_indicators.push_back(new_triangle);

        }
    }

}

