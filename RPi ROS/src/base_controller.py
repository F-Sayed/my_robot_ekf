#!/usr/bin/env python
from __future__ import print_function
import rospy
import qwiic_dual_encoder_reader
import time
import sys
import qwiic_scmd
from std_msgs.msg import String
from geometry_msgs.msg import Twist

R_MTR = 0
L_MTR = 1
FWD = 0
BWD = 1
myMotor = qwiic_scmd.QwiicScmd()
myEncoders = qwiic_dual_encoder_reader.QwiicDualEncoderReader()

def set_motor(leftDir, leftSpeed, rightDir, rightSpeed):
    myMotor.set_drive(R_MTR, rightDir, rightSpeed)
    myMotor.set_drive(L_MTR, leftDir, leftSpeed)

def forward():
    set_motor(FWD, 150, FWD, 150)

def stop():
    set_motor(FWD, 0, FWD, 0)


def reverse():
    set_motor(BWD, 150, BWD, 150)


def left():
    set_motor(FWD, 180, FWD, 150)


def left_reverse():
    set_motor(BWD, 180, BWD, 150)

def pivot_left():
    set_motor(BWD, 150, FWD, 150)


def right():
    set_motor(FWD, 180, FWD, 150)

def right_reverse():
    set_motor(BWD, 180, BWD, 150)

def pivot_right():
	set_motor(FWD, 150, BWD, 150)

#callback function from the keyboard teleop node
def set_cmd_vel(msg):
    linear_speed = msg.linear.x
    angular_speed = msg.angular.y
    #fwd
    if linear_speed > 0:
        #straight
        if angular_speed == 0:
            forward()
        #right
        elif angular_speed > 0:
            right()
        #left
        elif angular_speed < 0:
            left()

    #stationary
    elif linear_speed == 0:
        #no movement
        if angular_speed == 0:
            stop()
        #right
        elif angular_speed > 0:
            pivot_right()
        #left
        elif angular_speed < 0:
            pivot_left()
    
    #reverse
    elif linear_speed < 0:
        #straight
        if angular_speed == 0:
            reverse()
        #right
        elif angular_speed > 0:
            right_reverse()
        #left
        elif angular_speed < 0:
            left_reverse()
            
if __name__ == '__main__':                
    enc_pub = rospy.Publisher('encoder_values', String, queue_size=10)
    rospy.init_node('base_controller', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, set_cmd_vel)
    
    rate = rospy.Rate(30)
    
    e1 = myEncoders.count1
    e2 = myEncoders.count2
    #e1_prev = e1
    #e2_prev = e2
                
    while not rospy.is_shutdown():
        e1 = myEncoders.count1		#left
        e2 = myEncoders.count2		#right
        	
        speed_str = str(e1) + " " + str(e2)
        	
        enc_pub.publish(speed_str)
        	
        #e1_prev = e1
        #e2_prev = e2
        	
        rate.sleep()
