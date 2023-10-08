#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Export GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH="$SCRIPT_DIR/models:$GAZEBO_MODEL_PATH"
export GAZEBO_WORLD_PATH="$SCRIPT_DIR/worlds/fei_lrs_gazebo.world"
