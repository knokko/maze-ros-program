#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#java -jar packages/joystick/hello-cl.jar &
#java -jar packages/joystick/hello-cuda.jar &
#java -jar packages/joystick/hello-vulkan.jar &
cd /usr/local/cuda-10.2/
ls
cd /usr/local/cuda-10.2/bin
ls
echo includes
cd /usr/local/cuda-10.2/include
ls
cd /usr/local/cuda-10.2/lib64
ls
cd /usr/local/cuda-10.2/targets
ls


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
