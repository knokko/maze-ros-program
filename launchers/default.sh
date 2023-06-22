#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# echo "Running default.sh"
# roscore & # Do NOT use this when running on the duckiebot
# sleep 5

dt-exec roslaunch --wait agent agent_node.launch &
dt-launcher-joystick &
dt-launcher-camera_control


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
