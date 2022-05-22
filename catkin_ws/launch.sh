#!/bin/bash

if [[ "$#" != "2" ]]
then
    echo "using: $0 <script_name> <duckiebot_name>"
else
  . remote.sh $2
  . ./src/venv/bin/activate
  if [[ "$1" == "track" ]]
  then
      # shellcheck disable=SC2046
      dts duckiebot demo --demo_name indefinite_navigation --duckiebot `echo $VEHICLE_NAME`
  fi
    echo roslaunch autocalib $1.launch
    roslaunch autocalib $1.launch
fi