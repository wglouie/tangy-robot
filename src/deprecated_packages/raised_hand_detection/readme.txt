PACKAGE: raised_hand_detection

DEPENDENCY: 'openni_tracker_mod' package  (modified version of 'openni_tracker' package)
Download 'openni_tracker_mod' from https://drive.google.com/folderview?id=0B7mXbxbRVY1PTU1zWmJvN0dfcXM&usp=sharing

and 'users_markers' package
Download 'users_markers' package from https://drive.google.com/folderview?id=0B7mXbxbRVY1PRGo5WWlVZ0dIelU&usp=sharing


DESCRIPTION: This package has a node called 'raised_hand' that reads upper body markers for every detected users from topic /tracked_users_markers, 
published by 'users_markers' node, from 'users_markers' package. The 'raised_hand' node recognize users that are raising their hands and publishes
markers for this user on topic '/highlighted_user'.

SUBSCRIBES TO: '/tracked_users_markers'(type: visualization_msgs::Markers), published by 'users_markers' node from 'users_markers' package. 

PUBLISHES: '/highlighted_user' (type: visualization_msgs::Markers)


	DESCRIPTION: '/highlighted_user' publishes a pair of Markers that can be differentiated by their 'id'.

			If the marker id is equal to 0 (zero):
				Array of POINTS (one for every upper body user joint). 
				
				For instance:

						header: 
						  seq: 1684
						  stamp: 
						    secs: 1373577240
						    nsecs: 922791843
						  frame_id: /upper_frames/openni_depth_frame
						ns: raised_hand_detection
				>>>>>>>>>>>>>>> id: 0
						type: 8			<- It means that it is an array of POINTS
						action: 0
						pose: 
						  position: 
						    x: 0.0
						    y: 0.0
						    z: 0.0
						  orientation: 
						    x: 0.0
						    y: 0.0
						    z: 0.0
						    w: 0.0
						scale: 
						  x: 0.05
						  y: 0.05
						  z: 0.0
						color: 
						  r: 1.0
						  g: 0.0
						  b: 0.0
						  a: 1.0
						lifetime: 
						  secs: 0
						  nsecs: 0
						frame_locked: False
						points: 
						  - 			
						    x: 1.43039599991
						    y: 0.27957065331		<- LEFT HAND COORDINATES
						    z: 0.679397333754
						  - 
						    x: 1.59423918569
						    y: 0.0406490291856		<- LEFT ELBOW COORDINATES
						    z: 0.61919470556
						  - 
						    x: 1.44634863695
						    y: -0.11627285499		<- LEFT SHOULDER COORDINATES
						    z: 0.416570981214
						  - 
						    x: 1.42639203261
						    y: -0.272352335034		<- NECK COORDINATES
						    z: 0.409946420732
						  - 
						    x: 1.43185270027
						    y: -0.282168080357		<- HEAD COORDINATES
						    z: 0.625761830363
						  - 
						    x: 1.4064353062
						    y: -0.428431815079		<- RIGHT SHOULDER COORDINATES
						    z: 0.40332186025
						  - 
						    x: 1.39911136779
						    y: -0.494492510927		<- RIGHT ELBOW COORDINATES
						    z: 0.147721087591
						  - 
						    x: 1.27295362223
						    y: -0.471843926478		<- RIGHT HAND COORDINATES
						    z: -0.103852789828
						colors: []
						text: user 1			<- USER IDENTIFIER
						mesh_resource: ''
						mesh_use_embedded_materials: False
_______________________________________________________________________________________________________________________________
			If the marker id is equal to 1 (one):
				LINE STRIP (draw a line strip to connect the points from the previous marker). 
				
				For instance:

						header: 
						  seq: 1684
						  stamp: 
						    secs: 1373577240
						    nsecs: 922791843
						  frame_id: /upper_frames/openni_depth_frame
						ns: raised_hand_detection
				>>>>>>>>>>>>>>> id: 1
						type: 4			<- It means that it is a LINE STRIP
						action: 0
						pose: 
						  position: 
						    x: 0.0
						    y: 0.0
						    z: 0.0
						  orientation: 
						    x: 0.0
						    y: 0.0
						    z: 0.0
						    w: 0.0
						scale: 
						  x: 0.05
						  y: 0.05
						  z: 0.0
						color: 
						  r: 1.0
						  g: 0.0
						  b: 0.0
						  a: 1.0
						lifetime: 
						  secs: 0
						  nsecs: 0
						frame_locked: False
						points: 
						  - 			
						    x: 1.43039599991
						    y: 0.27957065331		<- LEFT HAND COORDINATES
						    z: 0.679397333754
						  - 
						    x: 1.59423918569
						    y: 0.0406490291856		<- LEFT ELBOW COORDINATES
						    z: 0.61919470556
						  - 
						    x: 1.44634863695
						    y: -0.11627285499		<- LEFT SHOULDER COORDINATES
						    z: 0.416570981214
						  - 
						    x: 1.42639203261
						    y: -0.272352335034		<- NECK COORDINATES
						    z: 0.409946420732
						  - 
						    x: 1.43185270027
						    y: -0.282168080357		<- HEAD COORDINATES
						    z: 0.625761830363
						  - 
						    x: 1.42639203261
						    y: -0.272352335034		<- NECK COORDINATES (AGAIN)
						    z: 0.409946420732
						  - 
						    x: 1.4064353062
						    y: -0.428431815079		<- RIGHT SHOULDER COORDINATES
						    z: 0.40332186025
						  - 
						    x: 1.39911136779
						    y: -0.494492510927		<- RIGHT ELBOW COORDINATES
						    z: 0.147721087591
						  - 
						    x: 1.27295362223
						    y: -0.471843926478		<- RIGHT HAND COORDINATES
						    z: -0.103852789828
						colors: []
						text: ''
						mesh_resource: ''
						mesh_use_embedded_materials: False

					 

			
