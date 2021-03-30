1) On tangy computer run: roslaunch bingo_game calibrate_kinect.launch
2) On remote computer run: roslaunch help_indicators calibate_kinect.launch
4) Place IR triangle ~1.15m (height) from base of the robot
5) Move robot using the "roslaunch help_indicators Calibrate_Kinect_bitches.launch" to a point where the triangle can be seen by the kinect
6) Retrieve the new values on the "rosrun help_indicators kinect_calibration" terminal by pressing enter
7) Follow steps 5 & 6 four times making sure to have many different XYZ positions
8) Retrieve the transform from "rosrun help_indicators kinect_calibration" terminal
9) Enter it into the desired file. In the Bingo case place it in BingoGameServer_remoteLaunch.
