#!/usr/bin/env python3

import rospy
import math

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
s
	# define a plan variable
	plan = Plan()
	
	plan_point1 = Twist()
	# just a quick solution to send two target points
	# define a point close to the initial position
	plan_point1.linear.x = -0.7924762782588125
	plan_point1.linear.y = -0.13300178332221238
	plan_point1.linear.z = 0.36339685365301155
	plan_point1.angular.x = 3.1415622780010195
	plan_point1.angular.y = -3.0205863424366773e-07
	plan_point1.angular.z = 1.5704225518606048
	# add this point to the plan
	plan.points.append(plan_point1)
	
	plan_point2 = Twist()
	# define a point away from the initial position
	plan_point2.linear.x = -0.6
	plan_point2.linear.y = -0.23
	plan_point2.linear.z = 0.25
	plan_point2.angular.x = 1.57
	plan_point2.angular.y = 0.0
	plan_point2.angular.z = 0.0
	# add this point to the plan
	plan.points.append(plan_point2)

	
	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
