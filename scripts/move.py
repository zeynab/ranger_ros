#!/usr/bin/env python

import time
from std_msgs.msg import *
from std_srvs.srv import Empty
import rospy

print("Use WASD to control the Ranger")

rospy.init_node('ranger_ctrl')
lmotor = rospy.Publisher('/lmotor', Float32)
rmotor = rospy.Publisher('/rmotor', Float32)

rospy.wait_for_service('halt')
halt = rospy.ServiceProxy('halt', Empty)

l = 0.0
r = 0.0

while True:
    key = raw_input("WASD?")

    if key.lower() == "w":
        l += 0.1
        r += 0.1
    elif key.lower() == "s":
        l -= 0.1
        r -= 0.1
    elif key.lower() == "a":
        l += 0.1
        r -= 0.1
    elif key.lower() == "d":
        r += 0.1
        l -= 0.1
    else:
        print("Stopping")
        r = 0.0
        l = 0.0

    print("L:%s      R:%s" % (l,r))

    lmotor.publish(Float32(l))
    rmotor.publish(Float32(r))


