#!/usr/bin/env python
# BEGIN ALL
import rospy
import functions
from geometry_msgs.msg import Pose

rospy.on_shutdown(functions.stop) # stop() will be called on shutdown
g_range_ahead = 1 # anything to start

rospy.init_node('explore_area_functions')
rate = rospy.Rate(1)

markers_total = 5
markers_pose_map = [None] * markers_total
markers_pose_next = [None] * markers_total
markers_pose_origin = [None] * markers_total
markers_letter = [None] * markers_total


# Main loop
while not rospy.is_shutdown():
  
  # Find the first marker
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

  # Decode the first message
  temp_qr_msg = functions.marker_decode()
  mark_no = temp_qr_msg[2]-1
  print("Found the marker no.: {}!".format(mark_no+1))
  markers_letter[mark_no] = temp_qr_msg[3]
  markers_pose_origin[mark_no] = temp_qr_msg[0]
  markers_pose_next[mark_no] = temp_qr_msg[1]

  # Adjust the marker position in the middle of the screen
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
      print("Turn right")
    else:
      if not last_turn:
        last_turn = True
        counter += 1
      functions.adjust(0.0, 0.2/counter)
      print("Turn left")
    rate.sleep()
  
  print("Success! Code is in the middle of the screen")

  markers_pose_map[mark_no] = functions.transpose_pose_rel(functions.pose_current(), functions.marker_pose_rel())
  print("Next marker in the radius: {}+-0.5".format(functions.dist2d(markers_pose_origin[mark_no],markers_pose_next[mark_no])))
  
  # # Go to the marker pose
  # functions.movebase_goal_execute(markers_pose_map[mark_no])
  # print("At the marker's position")
  
  rate.sleep()

# END ALL