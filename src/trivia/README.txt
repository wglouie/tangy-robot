Trivia Run Instructions

SETUP
1) Find out which eventID the keyboards are broadcasting on.
  -$cat /proc/bus/input/devices
  -find the devices with "Logitech in their names"
  -in the 'Handlers=sysrq kbd event__' field, note which number comes after the 'event' field (as of the last restart, the eventIDs were event23 and 14)
  -in 'trivia_game.launch', change the parameters evID0 and evID1 to have the right event numbers as discovered above
2) Map


RUNNING THE GAME
1st Terminal
-roslaunch tangy_bringup tangy_bringup.launch


2nd Terminal
-sudo su
-source /tangy-robot/devel/setup.bash
-roslaunch trivia trivia_game.launch

3rd Terminal (can be off the robot)
-rosrun trivia trivia_client


Trivia questions come from the database here: http://www.pubquizarea.com/multiple-choice/general-knowledge-quiz/start/&next=30/32/1/