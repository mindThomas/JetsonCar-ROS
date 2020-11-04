# Call this script by using "source PrepareHostROS.sh"
unset ROS_IP
unset ROS_HOSTNAME
unset ROS_MASTER_URI

export ROS_HOSTNAME=jetson-car.local
export ROS_MASTER_URI=http://jetson-car.local:11311
