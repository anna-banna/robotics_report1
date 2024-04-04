##ORIGINAL CODE FROM DR. H. SAEIDI WITH ADDITIONS PLAN_POINT3/4
#!/usr/bin/env python3

import rospy
import math

# import needed messages
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set frequency for loop
	loop_rate = rospy.Rate(10)

	# define plan variable
	plan = Plan()
	# define twist variable for first point 
	plan_point1 = Twist()

	# use rostopic echo /ur5e/toolpose to get linear and angular values
	# for safe initial position 
	plan_point1.linear.x = -0.7924762782588125
	plan_point1.linear.y = -0.13300178332221238
	plan_point1.linear.z = 0.36339685365301155
	plan_point1.angular.x = 3.1415622780010195
	plan_point1.angular.y = 0
	plan_point1.angular.z = 1.5704225518606048
	# add this point to the plan
	plan.points.append(plan_point1)

	# define twist variable for second point
	plan_point2 = Twist()
	# define a point under the initial position to pick up object
	plan_point2.linear.x = -0.7924762782588125
	plan_point2.linear.y = -0.13300178332221238
	plan_point2.linear.z = 0.1
	plan_point2.angular.x = 3.1415622780010195
	plan_point2.angular.y = 0
	plan_point2.angular.z = 1.5704225518
	# add this point to the plan
	plan.points.append(plan_point2)

	# define twist variable for third point
	plan_point3 = Twist()
	# define above the target position
	plan_point3.linear.x = -0.6729824076546461
	plan_point3.linear.y = 0.44900210793214634
	plan_point3.linear.z = 0.3501420732273582
	plan_point3.angular.x = 3.1415622780010195
	plan_point3.angular.y = 0
	plan_point3.angular.z = math.pi/4
	# add this point to the plan
	plan.points.append(plan_point3)

	# define twist variable for fourth point
	plan_point4 = Twist()
	# define a point as the target postion
	plan_point4.linear.x = -0.6729824076546461
	plan_point4.linear.y = 0.44900210793214634
	plan_point4.linear.z = 0.1
	plan_point4.angular.x = 3.1415622780010195
	plan_point4.angular.y = 0
	plan_point4.angular.z = math.pi/4
	# add this point to the plan
	plan.points.append(plan_point4)

	
	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
