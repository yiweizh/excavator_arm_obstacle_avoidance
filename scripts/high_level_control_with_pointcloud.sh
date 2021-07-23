#!/bin/sh

# scripts that publish aligned point clouds, do path planning and visualize the planning result

#
#xterm -e "roscore" &
#sleep 5
 
# publish pointclouds
xterm -e "rosrun motion_planning point_cloud_publisher.py" &
sleep 2

# open rviz to visualize the published data, remember to open the configuration after rviz is open
xterm -e "rosrun rviz rviz" &
sleep 2

# start path planning
xterm -e "rosrun motion_planning high_level_motion_planning.py" & 
sleep 2

# (optional) start rqt_gui, comment this line when low level control is started
xterm -e "rosrun rqt_gui rqt_gui" & 
