#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy

print("Use WASD to control the Ranger")

rospy.init_node('ranger_vw_ctrl')
twist = rospy.Publisher('/cmd_vel', Twist)

v = 0.0
w = 0.0

msg = Twist()

while True:
    key = raw_input("WASD?")

    if key.lower() == "w":
        v += 0.02
    elif key.lower() == "s":
        v -= 0.02
    elif key.lower() == "a":
        w += 0.02
    elif key.lower() == "d":
        w -= 0.02
    else:
        print("Stopping")
        v = 0.0
        w = 0.0

    print("V:%s      W:%s" % (v,w))

    msg.linear.x = v
    msg.angular.z = w
    twist.publish(msg)


