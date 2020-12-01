#!/usr/bin/env python
# BEGIN ALL
import rospy
import functions
import numpy
from geometry_msgs.msg import Pose

def patrol():
  if patrol.do_360:
    functions.adjust(0, numpy.pi/6)
    rospy.sleep(2)
    patrol.patrol_turn_counter += 1
    if patrol.patrol_turn_counter == 12:
      patrol.do_360 = False
      patrol.patrol_turn_counter = 0
  else:
    functions.patrol_movebase(patrol.patrol_positions_x[patrol.position_index], patrol.patrol_positions_y[patrol.position_index])
    patrol.position_index +=1
    if patrol.position_index == 7:
      patrol.position_index = 0
    patrol.do_360 = True
patrol.do_360 = True
patrol.patrol_turn_counter = 0
patrol.position_index = 0
patrol.patrol_positions_x = [-5, -5, 1, 4, 5, 5, 1, -5]
patrol.patrol_positions_y = [0, 2, 2, 2, 0, -2, -2, -2]

def rotate():
  if rotate.do_360:
    functions.adjust(0, numpy.pi/6)
    rospy.sleep(2)
    rotate.rotate_turn_counter += 1
    if rotate.rotate_turn_counter == 12:
      rotate.do_360 = False
      rotate.rotate_turn_counter = 0
  return rotate.do_360
rotate.do_360 = True
rotate.rotate_turn_counter = 0

rospy.on_shutdown(functions.stop) # stop() will be called on shutdown

rospy.init_node('explore_area_functions')
rate = rospy.Rate(1)

markers_total = 5
markers_pose_map = [None] * markers_total
markers_2d_origin = [None] * markers_total
markers_2d_next = [None] * markers_total
markers_2d_map = [None] * markers_total
markers_letter = [None] * markers_total

next_marker = 0
org_trans = None
last_err = 2

is_forced_to_patrol = False
#=====================
# Go to the center of the map
functions.patrol_movebase()
# MAIN LOOP
while not rospy.is_shutdown() and markers_letter.count(None) > 0:
  print("Message: {}".format(markers_letter))
  # If there are less than 2 decoded markers or robot is forced to patrol
  if markers_letter.count(None) > markers_total-2 or is_forced_to_patrol:
    # If there are no markers visible
    if not functions.marker_is_visible():
      patrol()
    # If the marker was already decoded and measured
    elif markers_letter[functions.marker_decode()[2]-1] != None:
      patrol()
    # Else decode new message
    else:
      temp_qr_msg = functions.marker_decode()
      print("Found new marker no.: {}!".format(temp_qr_msg[2]))

      # Adjust the marker position to the middle of the screen
      print("Aiming...")
      functions.aim()

      # Get information from the QR code
      if functions.marker_is_visible():
        temp_qr_msg = functions.marker_decode()
        mark_no = temp_qr_msg[2]-1

        # Save information only if it is a new marker
        if markers_letter[mark_no] == None:
          is_forced_to_patrol = False
          buf = mark_no + 1
          if buf > 4:
            buf -= markers_total
          while markers_letter[buf] != None:
            buf += 1
            if buf > 4:
              buf -= markers_total
          next_marker = buf - 1
          if next_marker < 0:
            next_marker += markers_total

          markers_letter[mark_no] = temp_qr_msg[3]
          markers_2d_origin[mark_no] = temp_qr_msg[0]
          markers_2d_next[mark_no] = temp_qr_msg[1]

          # Measure the marker's position in map's cooridnates
          markers_pose_map[mark_no] = functions.transpose_pose_rel(functions.pose_current(), functions.marker_pose_rel())
          markers_2d_map[mark_no] = (markers_pose_map[mark_no].position.x, markers_pose_map[mark_no].position.y)
          #print("Next marker in the radius: {}+-0.5".format(functions.dist2d(markers_2d_origin[mark_no],markers_2d_next[mark_no])))
        else:
          print("Marker {} was already decoded.".format(mark_no+1))
      else:
        print("Marker was lost.")

  # If there are at least 2 decoded markers
  else:
    # Estimate the pose of the marker's coordinate system in map's coordinates (needs at least 2 markers)
    new_trans, new_err = functions.estimate_org_pose(markers_2d_origin, markers_2d_map)
    new_err = abs(new_err-1)
    if org_trans == None or last_err > new_err:
      org_trans = new_trans
      last_err = new_err
    print("Estimated origin transposition (x, y, angle):\n{0}\nwith error: {1}".format(org_trans, last_err))

    target_marker = functions.org_to_map(markers_2d_next[next_marker], org_trans)

    if functions.marker_is_visible():
      if markers_letter[functions.marker_decode()[2]-1] == None:
        is_visible = True
      else:
        is_visible = False
    else:
      is_visible = False

    attempts = 0
    print("Scanning for the marker...")
    while not is_visible and attempts < 8:
      print("Attempt: {}".format(attempts))
      functions.go_to_marker(target_marker, (attempts+1)*45)
      rotate.do_360 = True
      rotate.rotate_turn_counter = 0
      while not is_visible and rotate():
        if functions.marker_is_visible():
          print("Seeing marker {0}, next marker {1}".format(functions.marker_decode()[2], next_marker+2))
          if markers_letter[functions.marker_decode()[2]-1] == None:
            is_visible = True
            print("Found new marker!")
      attempts += 1

    
    if functions.marker_is_visible():
      if markers_letter[functions.marker_decode()[2]-1] == None:
        temp_qr_msg = functions.marker_decode()
        print("Found new marker no.: {}!".format(temp_qr_msg[2]))

        if markers_letter.count(None) > 1:
          # Adjust the marker position to the middle of the screen
          print("Aiming...")
          functions.aim()

        # Get information from the QR code
        temp_qr_msg = functions.marker_decode()
        mark_no = temp_qr_msg[2]-1
        if markers_letter[mark_no] == None:
          is_forced_to_patrol = False

          if markers_letter.count(None) > 1:
            buf = mark_no + 1
            if buf > 4:
              buf -= markers_total
            while markers_letter[buf] != None:
              buf += 1
              if buf > 4:
                buf -= markers_total
            next_marker = buf - 1
            if next_marker < 0:
              next_marker += markers_total
          
          markers_letter[mark_no] = temp_qr_msg[3]
          markers_2d_origin[mark_no] = temp_qr_msg[0]
          markers_2d_next[mark_no] = temp_qr_msg[1]

          # Measure the marker's position in map's cooridnates
          markers_pose_map[mark_no] = functions.transpose_pose_rel(functions.pose_current(), functions.marker_pose_rel())
          markers_2d_map[mark_no] = (markers_pose_map[mark_no].position.x, markers_pose_map[mark_no].position.y)
        else:
          print("Marker {} was already decoded.".format(mark_no+1))
    else:
      is_forced_to_patrol = True
  
  rate.sleep()

print("Final message: {}".format("".join(markers_letter)))
# END ALL
