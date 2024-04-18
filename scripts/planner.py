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

pt_received = False
can_move = False
pt_in_camera = tf2_geometry_msgs.PointStamped() 
def getParams(data):
	global pt_received
	global pt_in_camera 
	pt_in_camera.point.x = data.xc
	pt_in_camera.point.y = data.yc 
	pt_in_camera.point.z = data.zc 
	pt_received = True 
	
def robotMove(data): 
	global can_move
	can_move = data.data

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# subscribe to sphere params 
	param_sub = rospy.Subscriber("/sphere_params", SphereParams, getParams)
	# subscribe to bool so I can tell it when to move 
	move_sub = rospy.Subscriber("/move_bool", Bool, robotMove)
	# set frequency for loop
	loop_rate = rospy.Rate(10)

	# define plan variable
	plan = Plan()
	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer) 
	
	
	while not rospy.is_shutdown():
		if pt_received and can_move:
			pt_in_camera.header.frame_id = 'camera_color_optical_frame'
			pt_in_camera.header.stamp = rospy.get_rostime() 
			pt_in_base = tfBuffer.transform(pt_in_camera, 'base', rospy.Duration(1.0))
	
			# define twist variable for first point 
			plan_point1 = Twist()
			plan_point1_mode = UInt8()
			# use rostopic echo /ur5e/toolpose to get linear and angular values
			# for safe initial position FOR THE REAL ROBOT
			plan_point1.linear.x = -0.01665031013035296
			plan_point1.linear.y = -0.4060903126050968
			plan_point1.linear.z = 0.42934662508780175
			plan_point1.angular.x = 3.1261380387567526
			plan_point1.angular.y = 0.01666427076882388
			plan_point1.angular.z = 1.530762598303694
			plan_point1_mode = 0
			# add this point to the plan
			plan.points.append(plan_point1)
			plan.modes.append(plan_point1_mode)
	
			#define point above ball 
			above_ball = Twist()
			above_ball_mode = UInt8()
			above_ball.linear.x = pt_in_base.point.x 
			above_ball.linear.y = pt_in_base.point.y
			above_ball.linear.z = 0.42934662508780175 
			above_ball.angular.x = 3.1261380387567526
			above_ball.angular.y = 0.01666427076882388
			above_ball.angular.z = 1.530762598303694
			above_ball_mode = 0
			plan.points.append(above_ball) 
			plan.modes.append(above_ball_mode)
	
			# move to ball without closing gripper
			on_ball = Twist()
			on_ball_mode = UInt8()
			on_ball.linear.x = pt_in_base.point.x 
			on_ball.linear.y = pt_in_base.point.y
			on_ball.linear.z = pt_in_base.point.z + 0.01
			on_ball.angular.x = 3.1261380387567526
			on_ball.angular.y = 0.01666427076882388
			on_ball.angular.z = 1.530762598303694
			on_ball_mode = 0
			plan.points.append(on_ball) 
			plan.modes.append(on_ball_mode)
			
			# close gripper
			on_ball2 = Twist()
			on_ball2_mode = UInt8()
			on_ball2.linear.x = pt_in_base.point.x 
			on_ball2.linear.y = pt_in_base.point.y
			on_ball2.linear.z = pt_in_base.point.z + 0.01
			on_ball2.angular.x = 3.1261380387567526
			on_ball2.angular.y = 0.01666427076882388
			on_ball2.angular.z = 1.530762598303694
			on_ball2_mode = 2
			plan.points.append(on_ball2)
			plan.modes.append(on_ball2_mode) 
			
			# move straight back up  
			above_ball_2 = Twist()
			above_ball_2_mode = UInt8()
			above_ball_2.linear.x = pt_in_base.point.x 
			above_ball_2.linear.y = pt_in_base.point.y
			above_ball_2.linear.z = 0.42934662508780175 
			above_ball_2.angular.x = 3.1261380387567526
			above_ball_2.angular.y = 0.01666427076882388
			above_ball_2.angular.z = 1.530762598303694
			above_ball_2_mode = 0
			plan.points.append(above_ball_2)
			plan.modes.append(above_ball_2_mode)

			# define twist variable for position over target
			plan_point3 = Twist()
			plan_point3_mode = UInt8()
			# define above the target position
			plan_point3.linear.x = 0.15
			plan_point3.linear.y = -0.4060903126050968
			plan_point3.linear.z = 0.42934662508780175
			plan_point3.angular.x = 3.1261380387567526
			plan_point3.angular.y = 0.01666427076882388
			plan_point3.angular.z = 1.530762598303694
			plan_point3_mode = 0
			# add this point to the plan
			plan.points.append(plan_point3)
			plan.modes.append(plan_point3_mode)
			
			# define twist variable for target point
			plan_point4 = Twist()
			plan_point4_mode = UInt8()
			# define a point as the target postion without opening the gripper
			plan_point4.linear.x = 0.15
			plan_point4.linear.y = -0.4060903126050968
			plan_point4.linear.z = pt_in_base.point.z + 0.01
			plan_point4.angular.x = 3.1261380387567526
			plan_point4.angular.y = 0.01666427076882388
			plan_point4.angular.z = 1.530762598303694
			plan_point4_mode = 0
			# add this point to the plan
			plan.points.append(plan_point4)
			plan.modes.append(plan_point4_mode)

			# define twist variable for target point
			plan_point4_2 = Twist()
			plan_point4_2_mode = UInt8()
			# define a point as the target postion and opening the gripper
			plan_point4_2.linear.x = 0.15
			plan_point4_2.linear.y = -0.4060903126050968
			plan_point4_2.linear.z = pt_in_base.point.z + 0.01
			plan_point4_2.angular.x = 3.1261380387567526
			plan_point4_2.angular.y = 0.01666427076882388
			plan_point4_2.angular.z = 1.530762598303694
			plan_point4_2_mode = 1
			# add this point to the plan
			plan.points.append(plan_point4_2)
			plan.modes.append(plan_point4_2_mode)
			
			# define twist variable for coming back up from target point
			plan_point5 = Twist()
			plan_point5_mode = UInt8()
			# define a point above target
			plan_point5.linear.x = 0.15
			plan_point5.linear.y = -0.4060903126050968
			plan_point5.linear.z = 0.42934662508780175
			plan_point5.angular.x = 3.1261380387567526
			plan_point5.angular.y = 0.01666427076882388
			plan_point5.angular.z = 1.530762598303694
			plan_point5_mode = 0
			# add this point to the plan
			plan.points.append(plan_point5)
			plan.modes.append(plan_point5_mode)
			
			# publish the plan
			plan_pub.publish(plan)
			# wait for 0.1 seconds until the next loop and repeat
			loop_rate.sleep()
