#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install preparation"

sudo apt-get -y install git

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core
echo "running the main install.sh"
./installation/install.sh

echo "clone simulation"
cd
git clone https://github.com/ctu-mrs/simulation.git
cd simulation
echo "rollback to commit with stable UVDAR"
git reset --hard 756dfa0e60eef0e92486496e8615d9801c8690b7
echo "running the main install.sh"
./installation/install.sh

echo "clone leader_follower_task"
cd
git clone https://github.com/ctu-mrs/leader_follower_task.git
cd leader_follower_task
echo "installing leader_follower_task submodules"
gitman install

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_core
ln -s ~/simulation
ln -s ~/leader_follower_task
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin init

echo "installation part ended"
