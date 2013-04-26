from std_msgs.msg import *
from std_srvs.srv import Empty
import rospy

from . import aseba

TICKS_PER_METER=69049
TICKS_TO_PID = 0.005124 # motor speed to send to go to 1 tick/sec

MAX_EYE = 40    # sym around 0
MAX_EYEB = 10   # sym around 0

# constants for LED effects
NONE           =0
GLOW_GREEN     =1
LEVEL_UP       =2
LEVEL_DOWN     =3
BLUSH          =4
PULSE_LOW_GREEN=5
PULSE_LOW_RED  =6
SPARKLE        =7

def clamp(val, minval, maxval):
    return max(min(val, maxval), minval)

# rosservice decorator
def rosservice(name, servicetype = Empty):
    def exportservice(fn):
        rospy.Service(name, servicetype, fn)

    return exportservice



class Ranger():
    def __init__(self):

        rospy.loginfo("Connecting to aseba network...")
        aseba.init()

        rospy.loginfo("Done. Loading Ranger scripts...")

        aseba.loadaesl("scripts/scenario1.aesl")

        self.lwheel_pub = rospy.Publisher('lwheel', Int16) # lwheel -> topic name
        aseba.subscribe('lwheel', self.onlwheel) # lwheel -> aseba event
        self.rwheel_pub = rospy.Publisher('rwheel', Int16)
        aseba.subscribe('rwheel', self.onrwheel)

        self.lmotor_sub = rospy.Subscriber('lwheel_vtarget', Float32, self.lmotor)
        self.rmotor_sub = rospy.Subscriber('rwheel_vtarget', Float32, self.rmotor)

        aseba.event("enable_torque", [1])

        rospy.loginfo("Done. Ranger is ready.")

    def onlwheel(self, msg):
        self.lwheel_pub.publish(-msg.data[0])

    def onrwheel(self, msg):
        self.rwheel_pub.publish(msg.data[0])

    def led(self):
        aseba.event("toto")

    def speedtopid(self, speed):
        ticks_sec = speed * TICKS_PER_METER
        return ticks_sec * TICKS_TO_PID

    def lmotor(self, speed):
        val = int(self.speedtopid(speed.data))
        rospy.loginfo("Setting mot1 PID target to %s" % val)
        aseba.set("mot1.pid.target_speed", -val, "smartrob3")

    def rmotor(self, speed):
        val = int(self.speedtopid(speed.data))
        rospy.loginfo("Setting mot2 PID target to %s" % val)
        aseba.set("mot2.pid.target_speed", val, "smartrob3")

    def heartbeat(self):
        """ Need to be called periodically (~50ms), else the robot
        turns off.
        """
        aseba.event("hearbeat")

    def eyes(self, e1, e2):
        """
        :param e1: first eye (value between -1.0 and 1.0)
        :param e2: second eye (value between -1.0 and 1.0)
        """
        aseba.event("set_eye", [int(clamp(e1 * MAX_EYE, -MAX_EYE, MAX_EYE)),
                                int(clamp(e2 * MAX_EYE, -MAX_EYE, MAX_EYE))])

    def eyebrows(self, eb1, eb2):
        """
        :param eb1: first eyebrow (value between -1.0 and 1.0)
        :param eb2: second eyebrow (value between -1.0 and 1.0)
        """
        aseba.event("set_eyebrow", [int(clamp(eb1 * MAX_EYEB, -MAX_EYEB, MAX_EYEB)),
                                int(clamp(eb2 * MAX_EYEB, -MAX_EYEB, MAX_EYEB))])


    @rosservice("halt")
    def halt(arg):
        aseba.event("motor_stop")

    @rosservice("leds/stop")
    def stop_leds(arg):
        aseba.event("set_led_effect", [NONE])
        return []

    @rosservice("leds/glow_green")
    def glow_green(arg):
        aseba.event("set_led_effect", [GLOW_GREEN])
        return []

    @rosservice("leds/level_up")
    def leds_level_up(arg):
        aseba.event("set_led_effect", [LEVEL_UP])
        return []

    @rosservice("leds/level_down")
    def leds_level_down(arg):
        aseba.event("set_led_effect", [LEVEL_DOWN])
        return []

    @rosservice("leds/pulse_green")
    def pulse_low_green(arg):
        aseba.event("set_led_effect", [PULSE_LOW_GREEN])
        return []

    @rosservice("leds/pulse_red")
    def pulse_low_red(arg):
        aseba.event("set_led_effect", [PULSE_LOW_RED])
        return []

    @rosservice("leds/blush")
    def blush(arg):
        aseba.event("set_led_effect", [BLUSH])
        return []

    @rosservice("leds/sparkle")
    def sparkle(arg):
        aseba.event("set_led_effect", [SPARKLE])
        return []
