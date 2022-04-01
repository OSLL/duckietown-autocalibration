#!/bin/bash
if [[ "$#" != "1" ]]
then
    echo "using: $0 <bot_name>"
else
	source devel/setup.bash

	export ROS_HOSTNAME=10.135.4.155
	export ROS_MASTER_URI=http://$1.local:11311
	export ROS_MASTER=$1
	export VEHICLE_NAME=$1
fi
