#! /usr/bin/env python3
# license removed for brevity
#******************************************************************************************************
# Author        : Jong-Wook Kim and Chien Van Dang 
# Updated date  : 2019-12-15
# Updated feature: User name is changed from "mija" to "Mija", and is registered to formal user list.
#                  User command is input in the next line starting "user: .."
#                  For the same robot posture, no waiting time before start. 
#                  If user makes a mistake like ordering to go to the current place, robot asks again.   
#******************************************************************************************************

import rospy, rospkg
import actionlib

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
from ar_track_alvar_msgs.msg import AlvarMarkers



import os, sys, time, _thread, sqlite3
import datetime
from datetime import date


rospack = rospkg.RosPack()
path = rospack.get_path('service')
place_info = path+"/params/place.splite"

class robotTravel():
	def __init__(self):
		self.goal_sent = False
		rospy.on_shutdown(self.shutdown)

		self.goal_name = "living room" 
		self.pose = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.pose.wait_for_server(rospy.Duration(5))

		rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.AR_callback)
		self.start = False

		
	def shutdown(self):
		if self.goal_sent:
			self.pose.cancel_goal()
		#print ("Mission completed\n")
		#rospy.sleep(1)

	def AR_callback(self, data):
		if data:
			self.hero_ar = data.markers[0].id
			print(self.hero_ar)
			self.start=True
		else:
			self.start=False

	def travel(self, goal_name):
		self.calGoalInfo(goal_name)
		tar_pos = self.goal_pos
		tar_quat = self.goal_quat
		#print(tar_pos)

		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'odom'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(tar_pos['x'], tar_pos['y'], 0.000), Quaternion(tar_quat['r1'], tar_quat['r2'], tar_quat['r3'], tar_quat['r4']))
		self.pose.send_goal(goal)


		success = self.pose.wait_for_result(rospy.Duration(1000))
		state = self.pose.get_state()
		result = False
		if success and state == GoalStatus.SUCCEEDED:
		     result = True
		else:
		     self.pose.cancel_goal()
		self.goal_sent = False

		return result

	def calGoalInfo(self, goal_name):
		connection = sqlite3.connect (place_info)
		cursor = connection.cursor()
		cursor.execute('SELECT * FROM %s' %(goal_name + "_info"))
		rows = cursor.fetchall()
		self.goal_pos = {'x': rows[0][2], 'y': rows[1][2]}
		self.goal_quat = {'r1': rows[3][2], 'r2': rows[4][2], 'r3': rows[5][2], 'r4': rows[6][2]}
		connection.close()

	def Run(self):
		if self.start is True:
			#epoch = rospy.Time() # secs=nsecs=0
			t = rospy.Time(5) # t.secs=10
			self.travel('place_%d'%self.hero_ar)

if __name__ == '__main__':
	try:
		rospy.init_node('service', anonymous=False)

		robot_navi = robotTravel()

		while not rospy.is_shutdown():
			robot_navi.Run()


				

	except rospy.ROSInterruptException:
		print ("Good bye!")

