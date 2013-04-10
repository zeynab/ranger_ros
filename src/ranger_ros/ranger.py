from std_msgs.msg import *
from std_srvs.srv import Empty
import rospy


from . import aseba

NEUIL_CODE ="""
onevent toto
call led.line(0,0,10,100,100)
call led.load(0)

"""

SMARTROB3_CODE ="""

onevent sensors.updated
emit lwheel mot1.enc._pulse[0]
emit rwheel mot2.enc._pulse[0]


onevent halt
mot1.pid.target_speed=0
mot2.pid.target_speed=0

"""

constants = {"NONE"           :0,
        "GLOW_GREEN"     :1,
        "LEVEL_UP"       :2,
        "LEVEL_DOWN"     :3,
        "BLUSH"          :4,
        "PULSE_LOW_GREEN":5,
        "PULSE_LOW_RED"  :6,
        "SPARKLE"        :7}


class Ranger():
    def __init__(self):

        rospy.loginfo("Connecting to aseba network...")
        aseba.init()

        rospy.loginfo("Done. Loading Ranger scripts...")

        self.lwheel_pub = rospy.Publisher('/lwheel', Int16)
        self.rwheel_pub = rospy.Publisher('/rwheel', Int16)

        self.lmotor_sub = rospy.Subscriber('/lmotor', Float32, self.lmotor)
        self.rmotor_sub = rospy.Subscriber('/rmotor', Float32, self.rmotor)

        rospy.Service('/halt', Empty, self.halt)

        #aseba.load(NEUIL_CODE, \
        #        node = "neuil", \
        #        events = {'toto':0}, \
        #        constants = constants)
        aseba.load(SMARTROB3_CODE, \
                node = "smartrob3", \
                events = {'lwheel':1,
                          'rwheel':1,
                          'halt':0})


        aseba.set("mot1.pid.enable", 2, "smartrob3")
        aseba.set("mot2.pid.enable", 2, "smartrob3")

        aseba.subscribe('lwheel', self.onlwheel)
        aseba.subscribe('rwheel', self.onrwheel)

    def onlwheel(self, msg):
        self.lwheel_pub.publish(-msg.data[0])

    def onrwheel(self, msg):
        self.rwheel_pub.publish(msg.data[0])

    def led(self):
        aseba.event("toto")

    def lmotor(self, speed):
        val = int(speed.data * 10)
        rospy.loginfo("Setting mot2 PID target to %s" % val)
        aseba.set("mot2.pid.target_speed", val, "smartrob3")

    def rmotor(self, speed):
        val = int(speed.data * 10)
        rospy.loginfo("Setting mot1 PID target to %s" % val)
        aseba.set("mot1.pid.target_speed", -val, "smartrob3")

    def halt(self, arg):
        rospy.logdebug("Tiggering event 'halt' on aseba network")
        aseba.set("mot1.pid.target_speed", 0, "smartrob3")
        aseba.set("mot2.pid.target_speed", 0, "smartrob3")
        #aseba.event("halt", "smartrob3")
        return []
