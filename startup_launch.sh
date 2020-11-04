#!/usr/bin/env bash
# Place the jetsoncar.service in /lib/systemd/system
# And enable the service on boot by running: 
#   sudo systemctl daemon-reload
#   sudo systemctl enable jetsoncar.service
# Make sure this file has correct permissions: chmod u+x startup_launch.sh

bash -c "source /opt/ros/melodic/setup.bash && source /home/jetson/ros_ws/devel/setup.bash && source /home/jetson/PrepareHostROS.sh && roslaunch jetsoncar_bringup minimal.launch > /home/jetson/startup_launch.log &"

#source /opt/ros/melodic/setup.bash 
#source /home/jetson/ros_ws/devel/setup.bash

#export ROS_IP=
#export ROS_HOSTNAME=jetson.local
#export ROS_MASTER_URI=http://jetson.local:11311

#roslaunch jetsoncar_bringup minimal.launch
