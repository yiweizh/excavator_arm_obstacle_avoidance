#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import time
from motion_planning.msg import excavator_control_signals # msg type for publishing data
from angle_subscriber import AngleSubscriber # get target excavator joint angles


def ExcavatorArmControl(boom_target,stick_target,boom_measured,stick_measured, tolerance = 2):
    # given the target boom, stick angle and the measured boom, stick angle
    # calculate desired stick servo angle, boom servo angle and pump mode
    # return [boom_servo, stick_servo, pump_mode]

    # initialize return values
    boom_servo = 90 # stop stick servo
    stick_servo = 90 # stop stick servo
    pump_mode = 0 # stop pump

    temp_boom_moving = False

    # control boom servo
    if abs(boom_target - boom_measured) > tolerance:
        # angle difference larger than tolerance
        # check whether we need to increase or decrease angle
        if boom_target > boom_measured: # increase angle
            boom_servo = 135
        else:
            boom_servo = 45
        temp_boom_moving = True
        pump_mode = 1 # normal opening

    if abs(stick_target - stick_measured) > tolerance and not temp_boom_moving:
        # angle difference larger than tolerance
        # check whether we need to increase or decrease angle
        if stick_target > stick_measured:
            stick_servo = 135
            pump_mode = 2 # increase stick angle, the motor may get stuck
        else:
            stick_servo = 45
            pump_mode = 1 # normal pump opening
        
    
    return [boom_servo, stick_servo, pump_mode]

class BaseControlPID(object):
    def __init__(self):
        self.initialize()

    def initialize(self):
        self.e2 = 0
        self.e1 = 0
        self.e0 = 0
        self.u2 = 0
        self.u1 = 0
        self.u0 = 0
        
        #PID constants
        Kp = 1.0
        Ki = 0.0
        Kd = 0.0

        N = 10
        Ts = 0.1 # unit: (s), rough estimate, may change later

        self.a0 = (1 + N*Ts)
        self.a1 = - (2 + N*Ts)
        self.a2 = 1.0
        self.b0 = Kp * (1 + N*Ts) + Ki * Ts * (1 + N*Ts) + Kd * N
        self.b1 = - (Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)
        self.b2 = Kp + Kd*N

        self.ku1 = self.a1 / self.a0
        self.ku2 = self.a2 / self.a0
        self.ke0 = self.b0 / self.a0
        self.ke1 = self.b1 / self.a0
        self.ke2 = self.b2 / self.a0

        self.max_pwm = 10
        self.min_pwm = 2

    def update(self, target_angle, measured_angle, tolerance = 2.0):
        angle_diff = target_angle - measured_angle

        if(abs(angle_diff) <= tolerance):
            self.initialize()
            return 0.0

        self.e2 = self.e1
        self.e1 = self.e0
        self.u2 = self.u1
        self.u1 = self.u0

        self.e0 = angle_diff
        self.u0 = -self.ku1 * self.u1 - self.ku2 * self.u2 + self.ke0 * self.e0 + self.ke1 * self.e1 + self.ke2 * self.e2

        ref_pwm = 0 # store reference, do not change u0
        
        if self.u0 > self.max_pwm:
            ref_pwm = self.max_pwm
        elif self.u0 < -self.max_pwm:
            ref_pwm = -self.max_pwm
        elif self.u0 > 0 and self.u0 < self.min_pwm:
            ref_pwm = self.min_pwm
        elif self.u0 < 0 and self.u0 > -self.min_pwm:
            ref_pwm = -self.min_pwm
        else:
            ref_pwm = self.u0

        return ref_pwm

def SimpleController():
    pub_p = rospy.Publisher('excavator_control', excavator_control_signals, queue_size=1)
    rospy.init_node('ExcavatorSimpleController', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #array = np.array([0,0,0])

    # get the target excavator joint angles
    target_angle_subscriber = AngleSubscriber(topic = 'target_angles')
    measured_angle_subscriber = AngleSubscriber(topic = 'measured_angles')
    base_control_pid = BaseControlPID()

    while not rospy.is_shutdown():
        #array += 1
        # get the current targets
        target_angles = target_angle_subscriber.get_angles()
        base_target = target_angles[0]
        boom_target = target_angles[1]
        stick_target = target_angles[2]

        # get the real angles
        measured_angles = measured_angle_subscriber.get_angles()
        base_measured = measured_angles[0]
        boom_measured = measured_angles[1]
        stick_measured = measured_angles[2]

        # create msg to pubslish
        control_signals = excavator_control_signals()


        # calculate the control signals
        
        # calculate excavator arm control values
        ret_val = ExcavatorArmControl(boom_target,stick_target,boom_measured,stick_measured)

        
        control_signals.boom_servo = ret_val[0]
        control_signals.stick_servo = ret_val[1]
        control_signals.pump_mode = ret_val[2]

        # calculate excavator base control values
    
        control_signals.base_pwm = base_control_pid.update(base_target,base_measured)
        

        
        pub_p.publish(control_signals)
        rate.sleep()

        
 
if __name__ == '__main__':
    try:
        SimpleController()
    except rospy.ROSInterruptException:
        pass
