#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
 
def scan_callback(msg, distance):
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  distance[0] = min(tmp)

def stop():
  rospy.Publisher('cmd_vel', Twist, queue_size=1).publish(Twist())
  print("Stopping")

rospy.on_shutdown(stop) # stop() will be called on shutdown
g_range_ahead = 1 # anything to start
dist = [None] # variable for the scan_callback()

rospy.init_node('wander')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback, dist)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

driving_forward = True
rate = rospy.Rate(60)

while not rospy.is_shutdown():
  if dist[0]!=None:
    g_range_ahead = dist[0]
  print g_range_ahead

  if g_range_ahead < 0.8:
    # TURN
    driving_forward = False
    print "Turn"
  else: # we're not driving_forward
    driving_forward = True # we're done spinning, time to go forward!
    #DRIVE
    print "Drive"
  
  twist = Twist()
  if driving_forward:
    twist.linear.x = 0.4
    twist.angular.z = 0.0
  else:
    twist.linear.x = 0.0
    twist.angular.z = 0.4
  cmd_vel_pub.publish(twist)
  
  rate.sleep()
# END ALL