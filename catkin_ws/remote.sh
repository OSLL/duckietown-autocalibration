#!/bin/bash
if [[ "$#" != "1" ]]
then
    echo "using: $0 <bot_name>"
else
	source ~/git/Software/catkin_ws/devel/setup.bash
	export ROS_MASTER_URI=http://$1:11311
	export ROS_MASTER=duckpi4
	export VEHICLE_NAME=duckpi4
fi
