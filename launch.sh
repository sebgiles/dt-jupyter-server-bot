#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
jupyter notebook --port=8888 --no-browser --ip=0.0.0.0 --allow-root
#roslaunch braitenberg_node braitenberg_node.launch veh:=$VEHICLE_NAME
