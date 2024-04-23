#!/usr/bin/env python3
##ORIGINAL CODE FROM DR. H. SAEIDI WITH ADDITIONS PLAN_POINT3/4

# import needed messages/packages
import rospy
import math
import tf2_ros
from tf.transformations import * 
import tf2_geometry_msgs
from robot_vision_lectures.msg import SphereParams
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

# set up flag to see if a target pt is received 
pt_received = False
# set up a flag to use for motion control
can_move = False
# set up flag to get tooltip's initial position
tooltip_pts_received = False
# set up flag to only print the plan once
plan_printed = False
# set up flag to print point in camera and base frame
pts_printed = False

# create variable for PointStamped msg type
pt_in_camera = tf2_geometry_msgs.PointStamped() 
# create variable for Twist msg type
tooltip_pts = Twist()

# check if plan is printed
def print_plan(plan):
	global plan_printed 
	if not plan_printed:
		print(plan)
		plan_printed = True
	
# check if point is printed 
def print_pts(camera, base):
	global pts_printed
	if not pts_printed:
		print("Point in camera frame: ") 
		print(camera)
		print("\nPoint in base frame: ")
		print(base)
		pts_printed = True

# get SphereParams
def getParams(data):
	global pt_received
	global pt_in_camera 
	pt_in_camera.point.x = data.xc
	pt_in_camera.point.y = data.yc 
	pt_in_camera.point.z = data.zc 
	pt_received = True 
	
# define function to tell robot when to move 
def robotMove(data): 
	global can_move
	can_move = data.data
	
# get tooltip's initial position
def tooltip_callback(data): 
	global tooltip_pts
	global tooltip_pts_received
	if not tooltip_pts_received:
		tooltip_pts.linear.x = data.linear.x 
		tooltip_pts.linear.y = data.linear.y
		tooltip_pts.linear.z = data.linear.z
		tooltip_pts.angular.x = data.angular.x
		tooltip_pts.angular.y = data.angular.y
		tooltip_pts.angular.z = data.angular.z
		tooltip_pts_received = True
	

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# subscribe to sphere params 
	param_sub = rospy.Subscriber("/sphere_params", SphereParams, getParams)
	# subscribe to toolpose to get accurate tooltip position 
	tooltip_sub = rospy.Subscriber("/ur5e/toolpose", Twist, tooltip_callback)
	# subscribe to bool so I can tell it when to move 
	move_sub = rospy.Subscriber("/move_bool", Bool, robotMove)
	# set frequency for loop
	loop_rate = rospy.Rate(10)

	# define plan variable
	plan = Plan()
	
	# set up buffer to transform point
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer) 
	
	
	while not rospy.is_shutdown():
		# build the plan if point is received
		if pt_received:
			# transform the point
			pt_in_camera.header.frame_id = 'camera_color_optical_frame'
			pt_in_camera.header.stamp = rospy.get_rostime() 
			pt_in_base = tfBuffer.transform(pt_in_camera, 'base', rospy.Duration(1.0))
	
			# define twist variable for first point 
			plan_point1 = Twist()
			plan_point1_mode = UInt8()
			# use rostopic echo /ur5e/toolpose to get linear and angular values
			# for safe initial position FOR THE REAL ROBOT
			plan_point1.linear.x = tooltip_pts.linear.x
			plan_point1.linear.y = tooltip_pts.linear.y
			plan_point1.linear.z = tooltip_pts.linear.z
			plan_point1.angular.x = tooltip_pts.angular.x
			plan_point1.angular.y = tooltip_pts.angular.y
			plan_point1.angular.z = tooltip_pts.angular.z
			# set mode
			plan_point1_mode.data = 0
			# add this point and mode to the plan
			plan.points.append(plan_point1)
			plan.modes.append(plan_point1_mode)
	
			# define point above ball 
			above_ball = Twist()
			above_ball_mode = UInt8()
			above_ball.linear.x = pt_in_base.point.x 
			above_ball.linear.y = pt_in_base.point.y
			above_ball.linear.z = tooltip_pts.linear.z 
			above_ball.angular.x = tooltip_pts.angular.x
			above_ball.angular.y = tooltip_pts.angular.y
			above_ball.angular.z = tooltip_pts.angular.z
			above_ball_mode.data = 0
			plan.points.append(above_ball) 
			plan.modes.append(above_ball_mode)
	
			# move to ball without closing gripper
			on_ball = Twist()
			on_ball_mode = UInt8()
			on_ball.linear.x = pt_in_base.point.x 
			on_ball.linear.y = pt_in_base.point.y
			on_ball.linear.z = pt_in_base.point.z + 0.01
			on_ball.angular.x = tooltip_pts.angular.x
			on_ball.angular.y = tooltip_pts.angular.y
			on_ball.angular.z = tooltip_pts.angular.z
			on_ball_mode.data = 0
			plan.points.append(on_ball) 
			plan.modes.append(on_ball_mode)
			
			# close gripper
			on_ball2 = Twist()
			on_ball2_mode = UInt8()
			on_ball2.linear.x = pt_in_base.point.x 
			on_ball2.linear.y = pt_in_base.point.y
			on_ball2.linear.z = pt_in_base.point.z + 0.01
			on_ball2.angular.x = tooltip_pts.angular.x
			on_ball2.angular.y = tooltip_pts.angular.y
			on_ball2.angular.z = tooltip_pts.angular.z
			on_ball2_mode.data = 2
			plan.points.append(on_ball2)
			plan.modes.append(on_ball2_mode) 
			
			# move straight back up  
			above_ball_2 = Twist()
			above_ball_2_mode = UInt8()
			above_ball_2.linear.x = pt_in_base.point.x 
			above_ball_2.linear.y = pt_in_base.point.y
			above_ball_2.linear.z = tooltip_pts.linear.z 
			above_ball_2.angular.x = tooltip_pts.angular.x
			above_ball_2.angular.y = tooltip_pts.angular.y
			above_ball_2.angular.z = tooltip_pts.angular.z
			above_ball_2_mode.data = 0
			plan.points.append(above_ball_2)
			plan.modes.append(above_ball_2_mode)

			# define twist variable for position over target
			plan_point3 = Twist()
			plan_point3_mode = UInt8()
			plan_point3.linear.x = 0.15
			plan_point3.linear.y = -0.4060903126050968
			plan_point3.linear.z = tooltip_pts.linear.z
			plan_point3.angular.x = tooltip_pts.angular.x
			plan_point3.angular.y = tooltip_pts.angular.y
			plan_point3.angular.z = tooltip_pts.angular.z
			plan_point3_mode.data = 0
			plan.points.append(plan_point3)
			plan.modes.append(plan_point3_mode)
			
			# define twist variable for target point
			plan_point4 = Twist()
			plan_point4_mode = UInt8()
			plan_point4.linear.x = 0.15
			plan_point4.linear.y = -0.4060903126050968
			plan_point4.linear.z = pt_in_base.point.z + 0.01
			plan_point4.angular.x = tooltip_pts.angular.x
			plan_point4.angular.y = tooltip_pts.angular.y
			plan_point4.angular.z = tooltip_pts.angular.z
			plan_point4_mode.data = 0
			plan.points.append(plan_point4)
			plan.modes.append(plan_point4_mode)

			# open the gripper
			plan_point4_2 = Twist()
			plan_point4_2_mode = UInt8()
			plan_point4_2.linear.x = 0.15
			plan_point4_2.linear.y = -0.4060903126050968
			plan_point4_2.linear.z = pt_in_base.point.z + 0.01
			plan_point4_2.angular.x = tooltip_pts.angular.x
			plan_point4_2.angular.y = tooltip_pts.angular.y
			plan_point4_2.angular.z = tooltip_pts.angular.z
			plan_point4_2_mode.data = 1
			plan.points.append(plan_point4_2)
			plan.modes.append(plan_point4_2_mode)
			
			# define twist variable for coming back up from target point
			plan_point5 = Twist()
			plan_point5_mode = UInt8()
			plan_point5.linear.x = 0.15
			plan_point5.linear.y = -0.4060903126050968
			plan_point5.linear.z = tooltip_pts.linear.z
			plan_point5.angular.x = tooltip_pts.angular.x
			plan_point5.angular.y = tooltip_pts.angular.y
			plan_point5.angular.z = tooltip_pts.angular.z
			plan_point5_mode.data = 0
			plan.points.append(plan_point5)
			plan.modes.append(plan_point5_mode)
			
			print_plan(plan)
			print_pts(pt_in_camera.point, pt_in_base.point)
			
			if can_move: 
				# publish the plan
				plan_pub.publish(plan)
				# wait for 0.1 seconds until the next loop and repeat
				loop_rate.sleep()

