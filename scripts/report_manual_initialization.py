#!/usr/bin/env python3
#ORIGINAL CODE FROM DR. H. SAEIDI WITH ADDITIONS AT LINE 42-45

import rospy
import math
# import the messages for reading the joint positions and sending joint commands
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header


if __name__ == '__main__':
	# initialize node
	rospy.init_node('report_manual_initialization', anonymous = True)
	# add publisher for sending joint position commands
	pos_pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size = 10)
	# set frequency for loop
	loop_rate = rospy.Rate(10)

	# define a joint trajectory variable for sending the control commands
	pos_cmd = JointTrajectory()
	pos_cmd_point = JointTrajectoryPoint()
	# complete message template
	pos_cmd.joint_names.append('elbow_joint')
	pos_cmd.joint_names.append('shoulder_lift_joint')
	pos_cmd.joint_names.append('shoulder_pan_joint')
	pos_cmd.joint_names.append('wrist_1_joint')
	pos_cmd.joint_names.append('wrist_2_joint')
	pos_cmd.joint_names.append('wrist_3_joint')
	
	# initialize the position command to zero
	for joint_no in range(6):
		pos_cmd_point.positions.append(0.0)
	# set the ideal time to destination
	pos_cmd_point.time_from_start = rospy.Duration(1.0) # here one second 
	# change the value of the command for the second joint
	pos_cmd_point.positions[1] = -math.pi/4
	# change the value of the command for the elbow joint
	pos_cmd_point.positions[0] = math.pi/4
	
	# change value of the command for the wrist joint 
	pos_cmd_point.positions[3] = -math.pi/2
	#change value of the command for the other wrist joint
	pos_cmd_point.positions[4] = -math.pi/2
	
	# add the trajectory point to command
	pos_cmd.points.append(pos_cmd_point)
	# define message header	
	header = Header()
	
	while not rospy.is_shutdown():
		# update header with the most recent time stamp
		header.stamp = rospy.Time.now()
		pos_cmd.header = header
		# publish the message
		pos_pub.publish(pos_cmd)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
