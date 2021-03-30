PACKAGE: users_markers

SPECIAL DEPENDENCY: openni_tracker_mod  (modified version of 'openni_tracker' package)
Download 'openni_tracker_mod' from https://drive.google.com/folderview?id=0B7mXbxbRVY1PTU1zWmJvN0dfcXM&usp=sharing

DESCRIPTION: This package has a node called 'users_markers' that sends upper body markers for every detected users.

SUBSCRIBES TO: '/tf'(type: tf::tfMessage) and '/users' (type: std_msgs/Int32MultiArray) , both topics published by 'openni_tracker' node from 'openni_tracker_mod' package. 

PUBLISHES: '/tracked_users_markers' (type: visualization_msgs::Markers)


		-markers.id = {user id}

		By reading the markers 'id' field, it is possible to identify which user is being represented.

		-markers.text = "user " + {user id}

		By reading the markers 'text' field, it is possible to identify which user is being represented.


		-markers.points = {user's upper body points}

			where:
				markers.points[0] = {left_hand}
				markers.points[1] = {left_elbow}
				markers.points[2] = {left_shoulder}
				markers.points[3] = {neck}
				markers.points[4] = {head}
				markers.points[5] = {right_shoulder}
				markers.points[6] = {right_elbow}
				markers.points[7] = {right_hand}




	

		
