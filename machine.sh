#!/usr/bin/env bash

source ~/tangy-robot/devel/setup.bash

#sets up environment variables
export ROSLAUNCH_SSH_UNKNOWN = 1
#export ROS_MASTER_URI=http://$(hostname).local:11311
export ROS_MASTER_URI=http://tangerine-tablet.local:11311
#export ROS_MASTER_URI=http://chris-XPS-8500.local:11311
#export ROS_MASTER_URI=http://geoff-N53SN.local:11311
#export ROS_MASTER_URI=http://asblab-Aspire-T3-100:11311
export ROS_HOSTNAME=$(hostname).local
export ROS_WORKSPACE=~/tangy-robot/


#updates the files
#roscd
#svn up
#catkin_make

exec "$@"
