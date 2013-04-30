import time
from std_msgs.msg import *
from std_srvs.srv import Empty
import rospy

from . import aseba

PREFIX="/home/lemaigna/src/ranger_ros/"

TICKS_PER_METER=69049
TICKS_TO_PID = 0.005124 # motor speed to send to go to 1 tick/sec

LEYE_OFFSET = 0 #23
REYE_OFFSET = 0  #63
LEYEB_OFFSET = 0
REYEB_OFFSET = 0

MAX_EYE = 100    # sym around 0
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

        aseba.loadaesl(PREFIX + "scripts/scenario1.aesl")

        self.lwheel_pub = rospy.Publisher('lwheel', Int16) # lwheel -> topic name
        aseba.subscribe('lwheel', self.onlwheel) # lwheel -> aseba event
        self.rwheel_pub = rospy.Publisher('rwheel', Int16)
        aseba.subscribe('rwheel', self.onrwheel)

        self.lmotor_sub = rospy.Subscriber('lwheel_vtarget', Float32, self.lmotor)
        self.rmotor_sub = rospy.Subscriber('rwheel_vtarget', Float32, self.rmotor)

        self.eyes(0,0)
        #self.calibrate_eyebrows()

        rospy.loginfo("Done. Ranger is ready.")

    def enable_motors(self):
        aseba.event("enable_torque", [1])

    def disable_motors(self):
        aseba.event("enable_torque", [0])


    def onlwheel(self, msg):
        self.lwheel_pub.publish(-msg.data[0])

    def onrwheel(self, msg):
        self.rwheel_pub.publish(msg.data[0])

    def led(self):
        aseba.event("toto")

    def _speedtopid(self, speed):
        ticks_sec = speed * TICKS_PER_METER
        return ticks_sec * TICKS_TO_PID

    def lmotor(self, speed):
        val = int(self._speedtopid(speed.data))
        rospy.loginfo("Setting mot1 PID target to %s" % val)
        aseba.set("mot1.pid.target_speed", -val, "smartrob3")

    def rmotor(self, speed):
        val = int(self._speedtopid(speed.data))
        rospy.loginfo("Setting mot2 PID target to %s" % val)
        aseba.set("mot2.pid.target_speed", val, "smartrob3")

    def shake(self, speed = 0.15):
        self.heartbeat()
        self.enable_motors()
        val = int(self._speedtopid(speed))
        aseba.set("mot1.pid.target_speed", val, "smartrob3")
        aseba.set("mot2.pid.target_speed", val, "smartrob3")
        time.sleep(0.5)
        aseba.set("mot1.pid.target_speed", -val, "smartrob3")
        aseba.set("mot2.pid.target_speed", -val, "smartrob3")
        time.sleep(0.5)
        aseba.set("mot1.pid.target_speed", val, "smartrob3")
        aseba.set("mot2.pid.target_speed", val, "smartrob3")
        time.sleep(0.5)
        aseba.set("mot1.pid.target_speed", -val, "smartrob3")
        aseba.set("mot2.pid.target_speed", -val, "smartrob3")
        time.sleep(0.5)
        aseba.set("mot1.pid.target_speed", 0, "smartrob3")
        aseba.set("mot2.pid.target_speed", 0, "smartrob3")
 


    def heartbeat(self):
        """ Need to be called periodically (~50ms), else the robot
        turns off.
        """
        aseba.event("hearbeat")

    def raweyes(self, eb1, eb2):
        """
        :param eb1: first eyebrow (value between -1.0 and 1.0)
        :param eb2: second eyebrow (value between -1.0 and 1.0)
        """
        rospy.logerr("Left: %s     Right: %s" % (eb1, eb2))
        aseba.event("set_eye", [eb1,eb2])


    def eyes(self, e1, e2):
        """
        :param e1: first eye (value between -1.0 and 1.0)
        :param e2: second eye (value between -1.0 and 1.0)
        """
        aseba.event("set_eye", [int(clamp(e1 * MAX_EYE, -MAX_EYE, MAX_EYE)) + LEYE_OFFSET,
                                int(clamp(e2 * MAX_EYE, -MAX_EYE, MAX_EYE)) + REYE_OFFSET])

    def eyebrow(self, eb1):
        """
        :param eb1: first eyebrow (value between -1.0 and 1.0)
        :param eb2: second eyebrow (value between -1.0 and 1.0)
        """
        print(eb1)
        aseba.event("set_eyebrow", [eb1,0])


    def calibrate_eyebrows(self):
        global LEYEB_OFFSET, REYEB_OFFSET
        eyebrows(-2, 2, clamped = False)
        time.sleep(4)
        eyebrows(-2, 2, clamped = False)
 


    def eyebrows(self, eb1, eb2, clamped = True):
        """
        :param eb1: first eyebrow (value between -1.0 and 1.0)
        :param eb2: second eyebrow (value between -1.0 and 1.0)
        """
        if clamped:
            l = int(clamp(eb1 * MAX_EYEB, -MAX_EYEB, MAX_EYEB)) + LEYEB_OFFSET
            r = int(clamp(eb2 * MAX_EYEB, -MAX_EYEB, MAX_EYEB)) + REYEB_OFFSET
        else:
            l = int(eb1 * MAX_EYEB) + LEYEB_OFFSET
            r = int(eb2 * MAX_EYEB) + REYEB_OFFSET

        aseba.event("set_eyebrow", [l,r])


    #@rosservice("halt")
    def halt(self, arg=None):
        aseba.event("motor_stop")

    #@rosservice("leds/stop")
    def stop_leds(self, arg=None):
        aseba.event("set_led_effect", [NONE])
        return []

    #@rosservice("leds/glow_green")
    def glow_green(self, arg= None):
        aseba.event("set_led_effect", [GLOW_GREEN])
        return []

    #@rosservice("leds/level_up")
    def leds_level_up(self, arg=None):
        aseba.event("set_led_effect", [LEVEL_UP])
        return []

    #@rosservice("leds/level_down")
    def leds_level_down(self, arg=None):
        aseba.event("set_led_effect", [LEVEL_DOWN])
        return []

    #@rosservice("leds/pulse_green")
    def pulse_low_green(self, arg=None):
        aseba.event("set_led_effect", [PULSE_LOW_GREEN])
        return []

    #@rosservice("leds/pulse_red")
    def pulse_low_red(self, arg=None):
        aseba.event("set_led_effect", [PULSE_LOW_RED])
        return []

    #@rosservice("leds/blush")
    def blush(self, arg=None):
        aseba.event("set_led_effect", [BLUSH])
        return []

    #@rosservice("leds/sparkle")
    def sparkle(self, arg=None):
        aseba.event("set_led_effect", [SPARKLE])
        return []
