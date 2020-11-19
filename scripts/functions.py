import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
scan_sub = [None]
dist = [None]
marker_msg_sub = [None]
marker_msg = [None]
marker_pos_sub = [None]

def scan_callback(msg, arg):
    tmp=[msg.ranges[0]]
    for i in range(1,21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
        tmp.append(msg.ranges[i])
    arg[0] = min(tmp)

def marker_msg_callback(msg, arg):
    arg[0] = msg.data

def distance():
    """
    Returns the distance to the closest object.
    """
    if scan_sub[0] == None:
        scan_sub[0] = rospy.Subscriber('scan', LaserScan, scan_callback, dist)
    while dist[0] == None:
        rospy.sleep(1)
    #sub.unregister()
    return dist[0]

def move(vel_forward, vel_angular):
    """
    move(vel_forward, vel_angular)
    """
    twist = Twist()
    twist.linear.x = vel_forward
    twist.angular.z = vel_angular
    vel_pub.publish(twist)
    #pub.unregister()

def stop():
    """
    Stops the robot.
    """
    vel_pub.publish(Twist())
    #pub.unregister()

def marker_is_visible():
    """
    Returns True if the marker is visible, otherwise False.
    """
    marker_msg[0] = None
    if marker_msg_sub[0] == None:
        marker_msg_sub[0] = rospy.Subscriber('/visp_auto_tracker/code_message', String, marker_msg_callback, marker_msg)
    while marker_msg[0] == None:
        rospy.sleep(1)
    if marker_msg[0] == "":
        return False
    else:
        return True

def marker_decode():
    """
    Returns [(x,y), (x_next,y_next), marker_no, letter]
    """
    if marker_msg_sub[0] == None:
        marker_msg_sub[0] = rospy.Subscriber('/visp_auto_tracker/code_message', String, marker_msg_callback, marker_msg)
    while marker_msg[0] == None:
        rospy.sleep(1)
    tmp = marker_msg[0].split("\r\n")
    for i, word in enumerate(tmp):
        tmp[i] = word.split("=")[1]
    return [(float(tmp[0]),float(tmp[1])), (float(tmp[2]),float(tmp[3])), int(tmp[4]), tmp[5]]