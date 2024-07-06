#!/bin/bash

#init var
WORKSPACE_DIR=~/ros2_ws
SRC_DIR=$WORKSPACE_DIR/src
PACKAGE_NAME=turtlesim_mo_ro
DOWNLOADS_DIR=~/Downloads
ZIP_FILE=$DOWNLOADS_DIR/turtlesim_mo_ro.zip
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

#mk workspace
mkdir -p $SRC_DIR

# dateien zu projekt
if [ -d $DOWNLOADS_DIR/$PACKAGE_NAME ]; then
    mv $DOWNLOADS_DIR/$PACKAGE_NAME $SRC_DIR/
fi

#Compile von pkg
cd $WORKSPACE_DIR
colcon build --packages-select $PACKAGE_NAME --allow-overriding turtlesim
if [ $? -eq 0 ]; then
    source install/setup.bash

    #settup script + starten von turtlesim_node
    gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    ros2 run turtlesim turtlesim_node
    " &
    sleep 4

    #run pkg
    gnome-terminal -- bash -c "
    source /opt/ros/humble/setup.bash
    source $WORKSPACE_DIR/install/setup.bash
    ros2 run $PACKAGE_NAME turtlesim_mo_ro
    "

    #rm 
    rm -rf $DOWNLOADS_DIR/$PACKAGE_NAME
else
    echo "Build failed. Please check the build logs for errors."
fi

