#!/bin/bash

# Init variables
WORKSPACE_DIR=~/ros2_ws
SRC_DIR=$WORKSPACE_DIR/src
PACKAGE_NAME=turtlesim_mo_ro
DOWNLOADS_DIR=~/Downloads
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

# Create workspace directory
mkdir -p $SRC_DIR

# Move package to the src directory
if [ -d $DOWNLOADS_DIR/$PACKAGE_NAME ]; then
    mv $DOWNLOADS_DIR/$PACKAGE_NAME $SRC_DIR/
fi

# Clone turtlesim if not already in the workspace
if [ ! -d $SRC_DIR/ros_tutorials ]; then
    cd $SRC_DIR
    git clone https://github.com/ros/ros_tutorials.git
fi

# Compile the package
cd $WORKSPACE_DIR
colcon build --packages-select $PACKAGE_NAME --allow-overriding turtlesim
if [ $? -eq 0 ]; then
    source install/setup.bash

    # Start turtlesim_node in a new terminal
    gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source $WORKSPACE_DIR/install/setup.bash
    ros2 run turtlesim turtlesim_node
    " &
    sleep 4

    # Run your package in another new terminal
    gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source $WORKSPACE_DIR/install/setup.bash
    ros2 run $PACKAGE_NAME turtlesim_mo_ro
    "

    # Clean up
    rm -rf $DOWNLOADS_DIR/$PACKAGE_NAME
else
    echo "Build failed. Please check the build logs for errors."
fi

