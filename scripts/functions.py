import math
import numpy
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
dist2 = 3*[None]

marker_msg_sub = [None]
marker_msg = [None]

marker_pose_sub = [None]
marker_pose = [None]

marker_status_sub = [None]

tf_listener = [None]
tf_broadcaster = [None]
vision_cone_angle = 90 #Must be divideable by 6
movebase_client = [None]
def scan_callback(msg, arg):
    tmp=[msg.ranges[0]]
    for i in range(1,21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
        tmp.append(msg.ranges[i])
    arg[0] = min(tmp)


def scan_callback_new(msg, arg):
    """
    Laser for left, center and right side of the vision cone.
    """
    tmp_left = []
    tmp_center=[msg.ranges[0]]
    tmp_right=[]
    #Left side of the vision code
    for i in range(len(msg.ranges)-(1+vision_cone_angle/2), len(msg.ranges)-(1+vision_cone_angle/6)):
        tmp_left.append(msg.ranges[i])
    #Center of the vision cone
    for i in range(1,(1+vision_cone_angle/6)):
        tmp_center.append(msg.ranges[i])
    for i in range(len(msg.ranges)-(1+vision_cone_angle/6), len(msg.ranges)):
        tmp_center.append(msg.ranges[i])
    #Right side of the vision cone
    for i in range(1+vision_cone_angle/6, 1+vision_cone_angle/2):
        tmp_right.append(msg.ranges[i])
    arg[0] = min(tmp_left)
    arg[1] = min(tmp_center)
    arg[2] = min(tmp_right)

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

def distance_new():
    """
    Returns the distance to the closest object for left, center and right side of the vision cone.
    """
    if scan_sub[0] == None:
        scan_sub[0] = rospy.Subscriber('scan', LaserScan, scan_callback_new, dist2)
    while dist2[0] == None:
        rospy.sleep(sleep_time)
    #sub.unregister()
    return dist2

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
    timeout = 0
    while marker_msg[0] == None and timeout < 100:
        rospy.sleep(sleep_time)
        timeout += 1
    if marker_msg[0] == "" or marker_msg[0] == None:
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
    pose = copy.deepcopy(parent_pose)
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
    pose.target_pose.header.stamp = rospy.Time.now()
    pose.target_pose.pose = goal_pose
    movebase_client[0].cancel_all_goals()
    movebase_client[0].stop_tracking_goal()
    movebase_client[0].send_goal(pose)
    print("[Movebase] New goal sent.")
    movebase_client[0].wait_for_result()
    print("[Movebase] Finished executing goal.")

def dist2d(pair1, pair2):
    """
    dist2d( (x1,y1), (x2,y2) )
    Returns distance between two points
    """
    return math.sqrt(math.pow(pair1[0]-pair2[0],2)+math.pow(pair1[1]-pair2[1],2))

def estimate_org_pose(org_poses, map_poses):
    """
    estimate_org_pose(org_poses, map_poses)
    Returns pose of origin coord system in map coord system.
    """
    A_arr = numpy.empty((0,4), float)
    B_arr = numpy.empty((0,1), float)

    for i in range(len(org_poses)):
        if org_poses[i] != None and map_poses[i] != None:
            A_arr = numpy.append(A_arr, numpy.array([[1, 0, org_poses[i][0], -org_poses[i][1]]]), axis=0)
            B_arr = numpy.append(B_arr, numpy.array([[map_poses[i][0]]]), axis=0)
            A_arr = numpy.append(A_arr, numpy.array([[0, 1, org_poses[i][1], org_poses[i][0]]]), axis=0)
            B_arr = numpy.append(B_arr, numpy.array([[map_poses[i][1]]]), axis=0)
    
    X_arr = numpy.linalg.lstsq(A_arr, B_arr)[0]
    angle = math.atan2(X_arr[3], X_arr[2])
    return (X_arr[0][0], X_arr[1][0], angle), math.pow(X_arr[2],2)+math.pow(X_arr[3],2)

def go_to_marker(marker_pose, angle):
    goal_pose = copy.deepcopy(marker_pose)
    goal_pose.position.x -= 1.7*math.cos(math.radians(angle))
    goal_pose.position.y -= 1.7*math.sin(math.radians(angle))
    print("New position:\n{}".format(goal_pose.position))
    movebase_goal_execute(goal_pose)

def patrol_movebase(pos_x = 0, pos_y = 0):
    goal_position = pose_current()
    goal_position.position.x = pos_x
    goal_position.position.y = pos_y
    movebase_goal_execute(goal_position)

def aim():
    counter = 1
    last_turn = False
    x_error = 1
    timeout = 0
    while x_error > 0.015 and counter < 4:
        while marker_is_visible() and x_error > 0.015 and counter < 4:
            if marker_pose_rel().position.x > 0:
                if last_turn:
                    last_turn = False
                    counter += 1
                adjust(0.0, -0.1/counter)
                print("[Aim] Adjusting right")
            else:
                if not last_turn:
                    last_turn = True
                    counter += 1
                adjust(0.0, 0.1/counter)
                print("[Aim] Adjusting left")
            rospy.sleep(1)
            if marker_is_visible():
                x_error = abs(marker_pose_rel().position.x)
        if not marker_is_visible():
            timeout += 1
            if timeout > 10:
                counter = 4
    
    print("[Aim] Success! Code is in the middle of the screen")

def org_to_map(marker_2d, org_trans):
    """
    org_to_map(marker_2d, org_trans)
    Returns transformed pose from original coords to map coords.
    """
    pose = pose_current()
    pose.position.x = org_trans[0] + marker_2d[0]*math.cos(org_trans[2]) - marker_2d[1]*math.sin(org_trans[2])
    pose.position.y = org_trans[1] + marker_2d[0]*math.sin(org_trans[2]) + marker_2d[1]*math.cos(org_trans[2])
    return pose
