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

		rospy.Subscriber("odom", Odometry, self.odom_callback)
		self.coordinate = Point()
		self.quaternion = Quaternion()

		
	def shutdown(self):
		if self.goal_sent:
			self.pose.cancel_goal()
		#print ("Mission completed\n")
		#rospy.sleep(1)

	def odom_callback(self, data):
		self.coordinate = data.pose.pose.position
		self.quaternion = data.pose.pose.orientation

	def updatePlaceTable(self, NewDestName):
		connection = sqlite3.connect(place_info)
		cursor = connection.cursor()
		#print("Connected to SQLite")
		print("cur_coord: (%.3f," %self.coordinate.x + "%.3f," %self.coordinate.y + "%.3f)" %self.coordinate.z)
		print("cur_quat: (%.3f," %self.quaternion.x + "%.3f," %self.quaternion.y + "%.3f," %self.quaternion.z + "%.3f)" %self.quaternion.w)

		cursor.executescript(""" DROP TABLE IF EXISTS %s_info;
						   CREATE TABLE %s_info(
				      	division STRING,
					position STRING,
					value DOUBLE);
					INSERT INTO %s_info(division, position, value) VALUES("coordinate", "x", 0.0);
					INSERT INTO %s_info(division, position, value) VALUES("coordinate", "y", 0.0);
					INSERT INTO %s_info(division, position, value) VALUES("coordinate", "z", 0.0);
					INSERT INTO %s_info(division, position, value) VALUES("quaternion", "x", 0.0);
					INSERT INTO %s_info(division, position, value) VALUES("quaternion", "y", 0.0);
					INSERT INTO %s_info(division, position, value) VALUES("quaternion", "z", 0.0);
					INSERT INTO %s_info(division, position, value) VALUES("quaternion", "w", 0.0)"""%(NewDestName, NewDestName, NewDestName, NewDestName, NewDestName, NewDestName, NewDestName, NewDestName, NewDestName))
		

		cursor.execute("UPDATE %s_info SET value= '%f' WHERE division = '%s' AND position = '%s'" %(NewDestName, self.coordinate.x, 'coordinate', 'x'))
		cursor.execute("UPDATE %s_info SET value= '%f' WHERE division = '%s' AND position = '%s'" %(NewDestName, self.coordinate.y, 'coordinate', 'y'))
		cursor.execute("UPDATE %s_info SET value= '%f' WHERE division = '%s' AND position = '%s'" %(NewDestName, self.coordinate.z, 'coordinate', 'z'))
		cursor.execute("UPDATE %s_info SET value= '%f' WHERE division = '%s' AND position = '%s'" %(NewDestName, self.quaternion.x, 'quaternion', 'x'))
		cursor.execute("UPDATE %s_info SET value= '%f' WHERE division = '%s' AND position = '%s'" %(NewDestName, self.quaternion.y, 'quaternion', 'y'))
		cursor.execute("UPDATE %s_info SET value= '%f' WHERE division = '%s' AND position = '%s'" %(NewDestName, self.quaternion.z, 'quaternion', 'z'))
		cursor.execute("UPDATE %s_info SET value= '%f' WHERE division = '%s' AND position = '%s'" %(NewDestName, self.quaternion.w, 'quaternion', 'w'))
		connection.commit()
		connection.close()

		#print("Record Updated successfully ")

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


if __name__ == '__main__':
	try:
		rospy.init_node('service', anonymous=False)

		robot_navi = robotTravel()

		while not rospy.is_shutdown():
			num = input(" 1. save place \n 2. navigation \n select(1/2): ")
			if num == "1":
				NewDest = input("Name: ")
				robot_navi.updatePlaceTable(NewDest)
			elif num == "2":
				setDestName = input("robot: Where would you like to go?\nuser: ")
				result = robot_navi.travel(setDestName)				
				if result:
					print ("robot: We have arrived!")
				else:
					print ("robot: Sorry, I missed the destination")
				

	except rospy.ROSInterruptException:
		print ("Good bye!")

