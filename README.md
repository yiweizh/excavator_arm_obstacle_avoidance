# excavator_arm_obstacle_avoidance
Capstone design project. Enable a model excavator arm from BuilderX to avoid obstacle autonomously.

## Perception Part
See [Perception](perception/readme.md)

## Installation

### Pre-requisite

Install ros_numpy

ROS Melodic:

`sudo apt-get install ros-melodic-ros-numpy`


## Quick Start

### Pre-requisite

Install xterm:

`sudo apt update`

`sudo apt install xterm`


Open a terminal in scripts folder, give permission to the ".sh" files:

`chmod +x low_level_control.sh high_level_control_with_pointcloud.sh`

### Usage 

To start low level control, open a terminal in scripts folder, enter

`./low_level_control.sh`

To start high level control only, start another terminal, enter

`roscore`

Then, open a terminal in scripts folder, enter

`./high_level_control_with_pointcloud`

Since high level control is used alone, remember to use the rqt GUI that poped out to set the "/measured_angles"

