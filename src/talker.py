#!/usr/bin/env python
import rospy

import sys
from geometry_msgs.msg import Twist

class talker(object):
	def __init__(self):
		self.pub = rospy.Publisher('cmdv', Twist, queue_size=10)
		rospy.init_node('cmd_talker', anonymous=True)
		rospy.loginfo('cmd_talker started...')
		self.twist = Twist()

	def cb_func(self, msg):
		rospy.loginfo(msg)

		self.twist.linear.x = msg.linear.z
            	self.twist.linear.y = 0
            	self.twist.linear.z = 0
            	self.twist.angular.x = 0
            	self.twist.angular.y = 0
            	self.twist.angular.z = msg.angular.z
		self.pub.publish(self.twist)	
		rospy.loginfo('-')

	def listener(self):
		rospy.Subscriber('/cmd_vel', Twist, self.cb_func)
		rospy.spin()

if __name__ == '__main__':
	t = talker()
	t.listener()
