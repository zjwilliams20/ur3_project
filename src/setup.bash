#!/bin/bash
# /src/setup.bash
# Run this before kicking off a gazebo simulation to tell it where to look for stuff.

PROJ_DIR="ur3_project"
cdir=$(pwd)

# Setup standard gazebo environment variables.
source /usr/share/gazebo/setup.sh

# Verify the current directory is ur3_project
shopt -s extglob           # enable +(...) glob syntax
result=${cdir%%+(/)}    # trim however many trailing slashes exist
result=${result##*/}       # remove everything before the last / that still remains

if [[ $result != $PROJ_DIR ]]; then
    echo "Error: setup.bash must be ran from ur3_project directory."
    return
fi

# Forward our custom paths to provide access to the models and world file.
export GAZEBO_MODEL_PATH=$cdir/submodules/models/
export GAZEBO_RESOURCE_PATH=$cdir/submodules/worlds/
