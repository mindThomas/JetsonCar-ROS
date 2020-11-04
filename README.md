# JetsonCar-ROS

## Install and Setup of nVidia Jetson TX2
1. Install Ubuntu 18.04 through JetPack 4.4.1.
2. Configure the PC name to be: `jetson-car`
3. Configure the username to be: `jetson`
3. Install ROS Melodic.


## Clone and Build
### Install necessary tools
This project uses ROS Melodic and the following extra dependencies:
```bash
sudo apt-get install python-catkin-tools
sudo apt-get install python-rosdep
```

### Cloning
To set up the simulation environment you need to clone the necessary repositories into an existing or new catkin workspace.
Follow the steps below to set up a new catkin workspace and clone:
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
catkin_init_workspace
git clone https://github.com/mindThomas/JetsonCar-Gazebo
git clone https://github.com/mindThomas/JetsonCar-ROS
git clone https://github.com/mindThomas/realsense_gazebo_plugin
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

### Building
Build the project with catkin build
```bash
cd ~/ros_ws
catkin build
source devel/setup.bash
```


# Notes
Descriptions and guides how to use this ROS project can be found in the notes below.

## USB rules file for automatic device detection
The MCU can automatically be detected and assigned to `/dev/jetsoncar` when connecting it over USB if the rules file, `99-jetsoncar.rules`, is installed.

To install the rules file, copy `99-jetsoncar.rules` to `/etc/udev/rules.d/`.


## Install as service on boot
A startup script and corresponding service (for starting at boot) for launching the minimal bringup launch file has been made.

Copy the `startup_launch.sh` and `PrepareHostROS.sh` to the home folder. Copy the file `jetsoncar.service` into `/lib/systemd/system` and modify the path to the `startup_launch.sh` script accordingly and rename the user and group to the username on the device if different. Enable the service on boot by running:
```bash
sudo systemctl daemon-reload
sudo systemctl enable jetsoncar.service
```

After the service has been installed the driver can be started, stopped or restarted by using:
```bash
sudo service jetsoncar start
sudo service jetsoncar stop
sudo service jetsoncar restart
```