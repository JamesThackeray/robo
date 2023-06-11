#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

class NavController():

	def initialize(self,x,y):
		rospy.init_node('navigate')
		pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
		rospy.sleep(2)
		checkpoint = PoseWithCovarianceStamped()
		checkpoint.header.frame_id = "map"
		checkpoint.pose.pose.position.x = x
		checkpoint.pose.pose.position.y = y
		checkpoint.pose.pose.position.z = 1.598
		
		[x,y,z,w]=quaternion_from_euler(0,0.0,0.0)
		checkpoint.pose.pose.orientation.x = x
		checkpoint.pose.pose.orientation.y = y
		checkpoint.pose.pose.orientation.z = 0.758
		checkpoint.pose.pose.orientation.w = 0.652

		# print checkpoint
		pub.publish(checkpoint)
		rospy.sleep(2)

	def move(self,goals):
		#tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("wait for the action server to come up")
		#allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(2))
		for g in goals:
			#we'll send a goal to the robot
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = g[0]
			goal.target_pose.pose.position.y = g[1]
			goal.target_pose.pose.position.z = 0
			[x,y,z,w]=quaternion_from_euler(0,0.0,0.0)
			goal.target_pose.pose.orientation.x = x
			goal.target_pose.pose.orientation.y = y
			if(g == goals[-2]):
				z = -0.844
				w = 0.537
			elif(g == goals[-3]):
				z = 0.919
				w = -0.395
			elif(g == goals[-1]):
				#z = 0.998
				#w = -0.059
				z = 1
				w = 0
			elif(g == goals[-4]):
				z = 0
				w = 0
			else:
				z = 1
			goal.target_pose.pose.orientation.z = z
			goal.target_pose.pose.orientation.w = w
			#start moving
			self.move_base.send_goal(goal)
			print("goal:{}".format(g))
			#allow TurtleBot up to 30 seconds to complete task
			if(g == goals[-1] or g == goals[-2] or g == goals[-3]):
				time = 15
			else:
				time = 10
			success = self.move_base.wait_for_result(rospy.Duration(time)) 

			if not success:
				self.move_base.cancel_goal()
				rospy.loginfo("What happenned bra")
			else:
				# We made it!
				state = self.move_base.get_state()
				if state == GoalStatus.SUCCEEDED:
					rospy.loginfo("Done moving")
		
def main():
	parser = argparse.ArgumentParser(description='Turtlebot Navigation Controller.')
	parser.add_argument('--org','--list', help='delimited list input', type=str,nargs="*")
	parser.add_argument('--dest','--list2', help='delimited list input', type=str,nargs="*")
	args = parser.parse_args()

	if(args.org):
		args.org = args.org[0].split(',')
		iniX = float(args.org[0])
		iniY = float(args.org[1])
	else:
		iniX = -2
		iniY = -1

	if(args.dest):
		all_goals = []
		for dest in args.dest:
			print dest
			coords = []
			goals = dest.split(',')
			print goals
			# for g in goals:
			x = float(goals[0])
			y = float(goals[1])
			coords.append(x)
			coords.append(y)
			all_goals.append(coords)
	else:
		all_goals = [[-3.7,0.087]]
	print(all_goals)
	# goals = [[-3.7,0.087],[-2.7,0.17]]
	nav = NavController()
	nav.initialize(iniX,iniY)
	nav.move(all_goals)
	

if __name__== "__main__":
  main()
