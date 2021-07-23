#!/bin/sh

# scripts that start low level controls of the excavator

#
xterm -e "roscore" &
sleep 5
 
# arduino interface that gets the base, boom and stick angles
xterm -e "rosrun motion_planning angle_publisher.py" &
sleep 2

# arduino interface that controls the pump, boom, stick servo and base 
xterm -e "rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB1" &
sleep 2

# start low level control, which receives a target profile containing base, boom and stick angles
# and publishes control signals
xterm -e "rosrun motion_planning excavator_simple_controller.py"
