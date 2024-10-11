#!/bin/bash

# -e: any subsequent commands which fail will cause the shell script to exit immediately
set -e

# use minimum PATH necessary
PATH=/sbin:/usr/sbin:/bin:/usr/bin

export LC_ALL=C    # Handle 8-bit characters e.g. ASCII copyright symbol

PROGNAME=`basename $0`
readonly DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Make sure kill signals kill the script, not some sub-shell cmd
trap "echo '${PROGNAME} killed with INT' ; exit 1" INT
trap "echo '${PROGNAME} killed with KILL' ; exit 1" KILL
trap "echo '${PROGNAME} killed with TERM' ; exit 1" TERM
trap "echo '${PROGNAME} killed with QUIT' ; exit 1" QUIT

usage_print () {
echo "Purpose:
Usage:
   ${PROGNAME} [-h] [-w <write-to-filename> ]
   Where:
      -h means print this help msg
      -l means write to this log file

   Example:
      ${PROGNAME} -h (print help msg)
      ${PROGNAME} -l <log filename>
"
}

# Clear out aliases that might affect how this script runs
do_clear_aliases () {
for VAR in `alias | sed 's,alias \([^=]*\).*,\1,'`
do
	unalias ${VAR}
done
}

## Clear out environment variables except what we use
do_clear_env_vars () {
VAR_KEEP="PATH PROGNAME PROGDIR TOPDIR DISPLAY HOME LANG LOGNAME PWD SHELL TERM TERMCAP USER VAR1 VAR_KEEP"
for VAR1 in `env -u "BASH_FUNC_which%%" | /usr/bin/cut -d= -f1`
do
	if [ -z "`echo ${VAR_KEEP} | grep ${VAR1}`" ] ; then
		unset ${VAR1}
	fi
done
unset VAR1
unset VAR_KEEP
}

do_clear_aliases
do_clear_env_vars

if [ "-h" = "${1}" -o "--help" = "${1}" ]; then
	usage_print
	exit 1
fi

echo -e "Building Jazzy ROS2 TurtleBot4 Environment..."
if [ -d src ]; then
	cd src
else
	mkdir src
fi

echo -e "Downloading iRobot Create3 Jazzy Base Packages..."
git clone https://github.com/iRobotEducation/create3_sim.git -b jazzy

echo -e "Downloading TurtleBot4 Jazzy Base..."
git clone https://github.com/turtlebot/turtlebot4.git -b jazzy

echo -e "Downloading TurtleBot4 Jazzy Simulator..."
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b jazzy

echo -e "Downloading TurtleBot4 Navigation2 Packages..."
git clone https://github.com/ros-navigation/navigation2.git -b jazzy
git clone https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation

echo -e "Installing base ROS packages..."
locale  # check for UTF-8

sudo apt update -y && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Enable Ubuntu Universe repo
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# Add ROS2 GPG key
sudo apt update -y && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repo to sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS and development tools sudo apt update && sudo apt install -y \
sudo apt-get install -y ros-dev-tools

# Install ROS-Jazzy with GUI tools
sudo apt install -y ros-jazzy-desktop

# Install Gazebo Harmonic and ROS-Gazebo bridge
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update -y
sudo apt-get install -y gz-harmonic
sudo apt-get install -y ros-jazzy-ros-gz

# Build ROS2 packages
rosdep init
rosdep install --from-path src -yi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Enable ROS2
echo -e "source /opt/ros/jazzy/setup.bash" >> /home/${USER}/.bashrc
echo -e "source ${DIR}/install/setup.bash" >> /home/${USER}/.bashrc
source /home/${USER}/.bashrc

