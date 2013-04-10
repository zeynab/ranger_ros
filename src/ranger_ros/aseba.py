
import tempfile

import roslib;roslib.load_manifest('ranger_ros')
from asebaros.srv import *
from asebaros.msg import *

import rospy

def init():
        try:
            rospy.wait_for_service('aseba/get_node_list', timeout=2)
        except rospy.ROSException:
            rospy.logerr("Could not connect to aseba! Did you start asebaros?")
            raise

def default_node():
    candidates = nodes()
    if not candidates:
        raise Exception("No aseba node currently available!")
    if len(candidates) > 1:
        raise Exception("You must select a node within %s" % candidates)
    return candidates[0]


def set(var, value, node = None):
    if not node:
        node = default_node()

    if not isinstance(value, list):
        value = [value]

    try:
        call = rospy.ServiceProxy('aseba/set_variable', SetVariable)
        return call(node, var, value)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def subscribe(name, cb):
    sub = rospy.Subscriber('/aseba/events/' + name, AsebaEvent, cb)


def event(name, data = []):
    pub = rospy.Publisher('/aseba/events/' + name, AsebaEvent)
    evt = AsebaEvent(stamp = rospy.Time.now(), source = 0, data = data)
    pub.publish(evt)

def nodes():
    try:
        call = rospy.ServiceProxy('aseba/get_node_list', GetNodeList)
        return call().nodeList
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def load(code, node = None, events = {}, constants = {}):
    if not node:
        node = default_node()

    rospy.logdebug("Uploading script to node <%s>" % node)
    with tempfile.NamedTemporaryFile(suffix='.aesl', delete=False) as tmp:
        tmp.write(b'<!DOCTYPE aesl-source>\n<network>\n')
        for k,v in events.items():
            tmp.write(b'<event size="%s" name="%s"/>\n' % (v,k))

        for k,v in constants.items():
            tmp.write(b'<constant value="%s" name="%s"/>\n' % (v,k))

        tmp.write(b'<node name="%s">\n' % node)
        tmp.write(code.replace('<', '&lt;').encode())
        tmp.write(b'\n</node>\n</network>')
        tmp.flush()
        try:
            call = rospy.ServiceProxy('aseba/load_script', LoadScripts)
            return call(tmp.name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


