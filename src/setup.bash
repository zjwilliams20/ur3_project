#!/bin/bash
# /src/setup.bash
# Run this before kicking off a gazebo simulation to tell it where to look for stuff.

# Setup standard gazebo environment variables.
source /usr/share/gazebo/setup.sh

# Forward our custom paths to provide access to the models and world file.
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Downloads/submodules/chesslab_setup/models/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/Downloads/submodules/chesslab_setup/worlds/
