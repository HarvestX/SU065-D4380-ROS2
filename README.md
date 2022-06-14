[![ci](https://github.com/HarvestX/SU065-D4380-ROS2/actions/workflows/ci.yml/badge.svg)](https://github.com/HarvestX/SU065-D4380-ROS2/actions/workflows/ci.yml)
[![lint](https://github.com/HarvestX/SU065-D4380-ROS2/actions/workflows/lint.yml/badge.svg)](https://github.com/HarvestX/SU065-D4380-ROS2/actions/workflows/lint.yml)

# SU065-D4380-ROS2
Nidec Motor Driver interface for ROS2

![Image](./media/SU065-M4380.jpg)

## Requirements
- Linux OS
  - [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- ROS 2
  - [Galactic Geochelone](https://index.ros.org/doc/ros2/Installation/Galactic/)

## Setup
Add user to dialout group.
```bash
sudo adduser $USER dialout
```
User will need to log out & log back in again for this to take effect.

## Install
### Locate package in workspace
```bash
mkdir -p ~/<Your Workspace>/src
cd ~/<Your Workspace>/src
git clone git@github.com:HarvestX/SU065-D4380-ROS2.git
```

### Install dependencies
```bash
source /opt/ros/galactic/setup.bash
cd ~/<Your Workspace>
rosdep update
rosdep install -r -y -i --from-paths ./src/SU065-D4380-ROS2 --rosdistro $ROS_DISTRO
```

### Build source
```bash
cd ~/<Your Workspace>
colcon build
```

# References
- [Nidec](https://www.nidec-shimpo.co.jp/en/)
