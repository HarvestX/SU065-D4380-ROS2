#!/usr/bin/env bash

if [ -z "$ROS_DISTRO" ]; then
  echo "Please set \$ROS_DISTRO variable."
  exit
fi

THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=$(realpath $(dirname $(realpath $THIS_FILE)))

# Install ROS2 dependency
## From git repos
colcon_ws=$(realpath ${THIS_PROJECT_ROOT}/../../)

## From apt repositories
rosdep update \
  --rosdistro $ROS_DISTRO
rosdep install -r -y -i \
  --from-paths ${colcon_ws}/src \
  --rosdistro $ROS_DISTRO

unset colcon_ws
unset THIS_FILE
unset THIS_PROJECT_ROOT
