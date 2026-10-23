#!/bin/bash
set -e

# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# If you built your own workspace inside the container, source it too
if [ -f "/workspace/install/setup.bash" ]; then
  source /workspace/install/setup.bash
fi

# Pass all arguments to CMD in Dockerfile or docker-compose
exec "$@"
