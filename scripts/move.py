#!/usr/bin/env python

import time
from std_msgs.msg import *
from std_srvs.srv import Empty
import rospy

print("Use WASD to control the Ranger")

rospy.init_node('ranger_ctrl')
lmotor = rospy.Publisher('/lwheel_vtarget', Float32)
rmotor = rospy.Publisher('/rwheel_vtarget', Float32)

l = 0.0
r = 0.0

while True:
    key = raw_input("QA WS?")

    if key.lower() == "q":
        l += 0.02
    elif key.lower() == "a":
        l -= 0.02
    elif key.lower() == "w":
        r += 0.02
    elif key.lower() == "s":
        r -= 0.02
    else:
        print("Stopping")
        r = 0.0
        l = 0.0

    print("L:%s m/s     R:%s m/s" % (l,r))

    lmotor.publish(Float32(l))
    rmotor.publish(Float32(r))


