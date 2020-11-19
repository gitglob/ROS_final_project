import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
scan_sub = [None]
dist = [None]

def scan_callback(msg, arg):
    tmp=[msg.ranges[0]]
    for i in range(1,21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
        tmp.append(msg.ranges[i])
    arg[0] = min(tmp)

def distance():
    if scan_sub[0] == None:
        scan_sub[0] = rospy.Subscriber('scan', LaserScan, scan_callback, dist)
    while dist[0]==None:
        rospy.sleep(1)
    #sub.unregister()
    return dist[0]

def move(vel_forward, vel_angular):
    twist = Twist()
    twist.linear.x = vel_forward
    twist.angular.z = vel_angular
    vel_pub.publish(twist)
    #pub.unregister()

def stop():
    vel_pub.publish(Twist())
    #pub.unregister()

def decode():
    pass
    #return [(x,y), (x_next,y_next), nr, letter]