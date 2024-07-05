#!/bin/bash

# Step 1: Set Variables
WORKSPACE_DIR=~/ros2_ws
SRC_DIR=$WORKSPACE_DIR/src
PACKAGE_NAME=turtlesim_automata
DOWNLOADS_DIR=~/Downloads
ZIP_FILE=$DOWNLOADS_DIR/turtlesim_mats.zip
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


# Step 2: Create workspace
mkdir -p $SRC_DIR

# Step 3: Move package to the appropriate folder
if [ -d $DOWNLOADS_DIR/$PACKAGE_NAME ]; then
    mv $DOWNLOADS_DIR/$PACKAGE_NAME $SRC_DIR/
    
# Step 4: Compile the package
cd $WORKSPACE_DIR
colcon build
if [ $? -eq 0 ]; then
    source install/setup.bash


# Step 5: Source the setup script
# Terminal 1: Start turtlesim_node
gnome-terminal -- bash -c 
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node &
sleep 4

# Step 6: Run the package
gnome-terminal -- bash -c 
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/install/setup.bash
ros2 run $PACKAGE_NAME turtlesim_automata


# Step 7: Clean up
rm -rf $DOWNLOADS_DIR/$PACKAGE_NAME


