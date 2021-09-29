#!/bin/bash

echo "Install curl"
sudo apt install -y curl

echo "Adding ROS repo"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo "Add keys"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "Update apt"
sudo apt update

echo "Install ROS noetic full"
sudo apt install -y ros-noetic-desktop-full

echo "Setting up environment"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc

echo "Installing dependencies"
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

echo "Initializing rosdep"
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update