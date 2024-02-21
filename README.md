Certainly! Here's the updated README with instructions on how to install rosdep:

# ROS Noetic Installation Guide for Ubuntu 20.04

This guide will walk you through the process of installing ROS Noetic on Ubuntu 20.04. ROS (Robot Operating System) is a flexible framework for writing robot software and is widely used in robotics research and development.

## Step 1: Set up your sources.list

Configure your Ubuntu repositories to allow the installation of packages from the ROS repository.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Step 2: Set up your keys

Add the ROS repository key to your system.

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

## Step 3: Update your package index

Update the package index to ensure that you have access to the latest available packages.

```bash
sudo apt update
```

## Step 4: Install ROS Noetic Desktop Full

Install the full desktop version of ROS Noetic, including simulation tools and other dependencies.

```bash
sudo apt install ros-noetic-desktop-full
```

## Step 5: Initialize rosdep

Before you can use ROS, you will need to initialize rosdep, which is a package manager for ROS dependencies.

```bash
sudo apt install python3-rosdep
rosdep update
sudo rosdep init
```

## Step 6: Set up environment variables

To ensure that ROS commands are available in your terminal, add the following line to your shell configuration file (e.g., ~/.bashrc for Bash users).

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Congratulations! You have successfully installed ROS Noetic on your Ubuntu 20.04 system and configured rosdep. You can now start using ROS for your robotics projects and experiments. Happy coding!


alias burger='export TURTLEBOT3_MODEL=burger'
alias waffle='export TURTLEBOT3_MODEL=waffle'
alias tb3fake='roslaunch turtlebot3_fake turtlebot3_fake.launch'
alias tb3teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
alias tb3='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias tb3maze='roslaunch turtlebot3_gazebo turtlebot3_world.launch'
alias tb3house='roslaunch turtlebot3_gazebo turtlebot3_house.launch'

export TURTLEBOT3_MODEL=waffle
export SVGA_VGPU10=0

