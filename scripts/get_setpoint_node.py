#!/usr/bin/env python
import rospy
from get_setpoints import dubins_velocity_publisher



rospy.init_node("setpoint_publish_node",anonymous=False)


rate = 100
loop_rate = rospy.Rate(rate)

velocity_publisher = dubins_velocity_publisher(rate)




while not rospy.is_shutdown():
    velocity_publisher.spin()
    loop_rate.sleep()
