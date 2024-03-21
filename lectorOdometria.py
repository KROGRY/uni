#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Callback function to handle odometry messages
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    linear_velocity = msg.twist.twist.linear
    angular_velocity = msg.twist.twist.angular

    print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z),end='\r')
    #print("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))
    #print("Linear Velocity: x={}, y={}, z={}".format(linear_velocity.x, linear_velocity.y, linear_velocity.z))
    #print("Angular Velocity: x={}, y={}, z={}".format(angular_velocity.x, angular_velocity.y, angular_velocity.z))

def odom_listener():
    # ROS node initialization
    rospy.init_node('odom_listener_Bruno', anonymous=True)

    # Subscriber to the odometry topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Spin to keep the script alive
    rospy.spin()
    rospy.rate=Rate(1)

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass
