import math
import copy
import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Int8

sleep_time = 1

vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

scan_sub = [None]
dist = [None]

marker_msg_sub = [None]
marker_msg = [None]

marker_pose_sub = [None]
marker_pose = [None]

marker_status_sub = [None]

tf_listener = [None]
tf_broadcaster = [None]

movebase_client = [None]

def scan_callback(msg, arg):
    tmp=[msg.ranges[0]]
    for i in range(1,21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
        tmp.append(msg.ranges[i])
    arg[0] = min(tmp)

def marker_msg_callback(msg, arg):
    arg[0] = msg.data

def marker_pose_callback(msg, arg):
    arg[0] = msg.pose

def distance():
    """
    Returns the distance to the closest object.
    """
    if scan_sub[0] == None:
        scan_sub[0] = rospy.Subscriber('scan', LaserScan, scan_callback, dist)
    while dist[0] == None:
        rospy.sleep(sleep_time)
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

def adjust(vel_forward, vel_angular, dtime = 1):
    """
    adjust(vel_forward, vel_angular, dtime = 1)
    Works like move() but sets the speed for limited amount of time.
    """
    twist = Twist()
    twist.linear.x = vel_forward
    twist.angular.z = vel_angular
    vel_pub.publish(twist)
    rospy.sleep(dtime)
    stop()
    #pub.unregister()

def stop():
    """
    Stops the robot.
    """
    vel_pub.publish(Twist())
    #pub.unregister()

def pose_current(child_frame = '/base_footprint', parent_frame = '/map'):
    """
    pose_current(child_frame = '/base_footprint', parent_frame = '/map')
    Returns current robot pose in map coordinates by default.
    """
    if tf_listener[0] == None:
        tf_listener[0] = tf.TransformListener()
    pose = Pose()
    while pose == Pose():
        try:
            (pose.position, pose.orientation) = tf_listener[0].lookupTransform(parent_frame, child_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Something went horribly wrong...")
    pose.position = Point(*pose.position)
    pose.orientation = Quaternion(*pose.orientation)
    return pose

def marker_is_onscreen():
    """
    THIS WORKS ONLY ONCE SMH
    Returns True if the marker is in the frame, otherwise False.
    """
    marker_status = [None]
    if marker_status_sub[0] == None:
        marker_status_sub[0] = rospy.Subscriber('/visp_auto_tracker/status', Int8, marker_msg_callback, marker_status)
    counter = 0
    while marker_status[0] == None:
        counter += 1
        if counter > 2:
            marker_status[0] = 1
            print("Timeout")
        else:
            rospy.sleep(sleep_time)
            print("Missing message")
    
    if marker_status[0] == 1:
        return False
    else:
        return True

def marker_is_visible():
    """
    Returns True if the marker is visible, otherwise False.
    """
    marker_msg[0] = None
    if marker_msg_sub[0] == None:
        marker_msg_sub[0] = rospy.Subscriber('/visp_auto_tracker/code_message', String, marker_msg_callback, marker_msg)
    while marker_msg[0] == None:
        rospy.sleep(sleep_time)
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
        rospy.sleep(sleep_time)
    tmp = marker_msg[0].split("\r\n")
    for i, word in enumerate(tmp):
        tmp[i] = word.split("=")[1]
    return [(float(tmp[0]),float(tmp[1])), (float(tmp[2]),float(tmp[3])), int(tmp[4]), tmp[5]]

def transpose_pose_rel(parent_pose, child_pose):
    """
    Returns trasposed marker's pose related to the map.
    Marker must be in the center of the screen.
    """
    pose = copy.copy(parent_pose)
    angle = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
    pose.position.x += child_pose.position.z * math.cos(angle)
    pose.position.y += child_pose.position.z * math.sin(angle)
    return pose


def marker_pose_rel():
    """
    Returns marker's pose related to the camera.
    """
    if marker_pose_sub[0] == None:
        marker_pose_sub[0] = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, marker_pose_callback, marker_pose)
    while marker_pose[0] == None:
        rospy.sleep(sleep_time)
    return marker_pose[0]

def transform_publish(tf_pose, child_frame, parent_frame = '/base_footprint'):
    """
    This doesnt work very well...
    """
    if tf_broadcaster[0] == None:
        tf_broadcaster[0] = tf.TransformBroadcaster()
    
    translation = (
    tf_pose.position.x,
    tf_pose.position.y,
    tf_pose.position.z)
    rotation = (
    tf_pose.orientation.x,
    tf_pose.orientation.y,
    tf_pose.orientation.z,
    tf_pose.orientation.w)

    tf_broadcaster[0].sendTransform(translation, rotation, rospy.Time.now(), child_frame, parent_frame)
    if tf_listener[0] == None:
        tf_listener[0] = tf.TransformListener()
    pose = Pose()
    while pose == Pose():
        try:
            (pose.position, pose.orientation) = tf_listener[0].lookupTransform(parent_frame, child_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Publishing the pose again...")
            tf_broadcaster[0].sendTransform(translation, rotation, rospy.Time.now(), child_frame, parent_frame)


def movebase_goal_execute(goal_pose):
    """
    Sets the goal pose in map's coordinates.
    """
    if movebase_client[0] == None:
        movebase_client[0] = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    movebase_client[0].wait_for_server()
    pose = MoveBaseGoal()
    pose.target_pose.header.frame_id = "map"
    pose.target_pose.pose = goal_pose
    movebase_client[0].send_goal(pose)
    movebase_client[0].wait_for_result()

def dist2d(pair1, pair2):
    return math.sqrt(math.pow(pair1[0]-pair2[0],2)+math.pow(pair1[1]-pair2[1],2))