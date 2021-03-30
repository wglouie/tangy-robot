#!/bin/bash
# Run Script
# chmod 755 run

# pkill ros

echo "running my_robot_configuration"
gnome-terminal -e roslaunch tangerine_2dnav my_robot_configuration.launch & # &> my_robot_config_debug.txt
echo "Sleeping for 5 seconds"
sleep 5s
echo "running move_base"
gnome-terminal -e roslaunch tangerine_2dnav move_base.launch & #&> move_base_debug.txt
echo "Sleeping for 5 seconds"
sleep 5s
echo "running BingoGameServer"
gnome-terminal -e roslaunch bingo_game BingoGameServer.launch & #&> BingoGameServer_debug.txt
echo "Sleeping for 1 seconds"
sleep 1s
echo "running help_light"
gnome-terminal -e rosrun help_indicators help_light & #&> help_light_debug.txt
echo "Sleeping for 1 seconds"
sleep 1s
echo "running bingo_game_client"
gnome-terminal -e rosrun bingo_game bingo_game_client & #&> bingo_game_client_debug.txt
echo "Sleeping for 1 seconds"
sleep 1s
echo "running rviz"
gnome-terminal -e rviz & #&> rviz_debug.txt
