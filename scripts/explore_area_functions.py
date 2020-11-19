#!/usr/bin/env python
# BEGIN ALL
import rospy
import functions
from geometry_msgs.msg import Pose

rospy.on_shutdown(functions.stop) # stop() will be called on shutdown
g_range_ahead = 1 # anything to start

rospy.init_node('explore_area_functions')
rate = rospy.Rate(1)

while not rospy.is_shutdown():
  
  while not functions.marker_is_visible():
    g_range_ahead = functions.distance()
    print g_range_ahead
    if g_range_ahead < 1.2:
      functions.move(0.0, 0.2)
      print "Turn"
    else:
      functions.move(0.4, 0.0)
      print "Drive"
    
    rate.sleep()

  print("Found the marker!")
  print(functions.marker_decode())

  print("Aiming...")
  counter = 1
  last_turn = False
  while functions.marker_is_visible() and abs(functions.marker_pose_rel().position.x) > 0.02:
    if functions.marker_pose_rel().position.x > 0:
      print(functions.marker_pose_rel().position.x)
      if last_turn:
        last_turn = False
        counter += 1
      functions.adjust(0.0, -0.2/counter)
      print "Turn right"
    else:
      if not last_turn:
        last_turn = True
        counter += 1
      functions.adjust(0.0, 0.2/counter)
      print "Turn left"
  print("Success")
  marker_pos_rel = functions.marker_pose_rel()
  print("Markers rel pose: {}".format(marker_pos_rel))
  pose = Pose()
  pose.position.x = marker_pos_rel.position.z
  pose.orientation.w = 1.0
  print("publishing pose..")
  functions.transform_publish(pose, '/temp_marker')
  #print("getting pose..")
  #marker_pose = functions.pose_current('/temp_marker')
  #print("Markers pose in map: {}".format(marker_pose))
  


  rate.sleep()

# END ALL