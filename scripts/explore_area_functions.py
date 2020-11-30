#!/usr/bin/env python
# BEGIN ALL
import rospy
import functions
import numpy
from geometry_msgs.msg import Pose

def patrol():
  g_range_ahead = functions.distance()
  print("Laser distance: {}".format(g_range_ahead))
  if g_range_ahead[1] < 1.2:
    functions.move(0.0, 0.2)
    print("Turn")
  else:
    functions.move(0.4, 0.0)
    print("Drive")
  rospy.sleep(1)

def patrol_movebase(pos_x, pos_y):
  goal_position = functions.pose_current()
  print("Robot position{} \n".format(goal_position))
  goal_position.position.x = pos_x
  goal_position.position.y = pos_y
  print("Goal position{} \n".format(goal_position))
  functions.movebase_goal_execute(goal_position)
  rospy.sleep(5)

def patrol_go_to_origin():
  goal_position = functions.pose_current()
  print("Robot position{}".format(goal_position))
  goal_position.position.x = 0
  goal_position.position.y = 0
  print("Goal position{}".format(goal_position))
  functions.movebase_goal_execute(goal_position)
  rospy.sleep(1)

def rotate():
  # Setting the current time for distance calculus
  t0 = rospy.Time.now().to_sec()
  current_angle = 0
  relative_angle = 360
  while(current_angle < relative_angle):
    if not functions.marker_is_visible():
    # Rotating 360 degrees
      functions.adjust(0.0, -(360*3.14/360))
      t1 = rospy.Time.now().to_sec()
      current_angle = -(360*3.14/360)*(t1-t0)
    else:
      current_angle = relative_angle



def go_to_marker(markers_pose_map):
	if markers_letter.count(None) == 2:
		markers_pose_map[mark_no].position.x - 7
		markers_pose_map[mark_no].position.y - 7
		functions.movebase_goal_execute(markers_pose_map[mark_no])

		t_0 = rospy.Time.now().to_sec()
		distance_to_travel = 3
		current_distance = 0
		while current_distance < distance_to_travel:
			if not functions.marker_is_visible():
				t_1 = rospy.Time.now().to_sec()
				current_distance = 0.4*(t_1 - t_0)
				functions.move(0.4, 0.0)
				rotate()
			else:
				current_distance = distance_to_travel

rospy.on_shutdown(functions.stop) # stop() will be called on shutdown
g_range_ahead = 1 # anything to start



rospy.init_node('explore_area_functions')
rate = rospy.Rate(1)

markers_total = 5
markers_pose_map = [None] * markers_total
markers_2d_origin = [None] * markers_total
markers_2d_next = [None] * markers_total
markers_2d_map = [None] * markers_total
markers_letter = [None] * markers_total
# Patrol_new variables
patrol_turn_counter = 0
patrol_go_to_origin()
do_360 = True
position_index = 0
first_marker_is_found = False
second_marker_is_found = False
patrol_positions_x = [-5, -5, 1, 4, 5, 5, 1, -5]
patrol_positions_y = [0, 2, 2, 2, 0, -2, -2, -2]
#=====================
# MAIN LOOP
#patrol_movebase(functions.pose_current())
while not rospy.is_shutdown():
  # if there are no markers or the marker was already decoded and measured, then patrol
  if not functions.marker_is_visible() or not markers_letter[functions.marker_decode()[2]-1] == None:
    if do_360:
      functions.adjust(0, numpy.pi/4)
      patrol_turn_counter += 1
      if patrol_turn_counter == 8:
        do_360 = False
        patrol_turn_counter = 0
    else:
      patrol_movebase(patrol_positions_x[position_index], patrol_positions_y[position_index])
      position_index +=1
      if position_index == 7:
        position_index = 0
      do_360 = True
  elif markers_letter[functions.marker_decode()[2]-1] == None:
    # Decode new message
    temp_qr_msg = functions.marker_decode()
    mark_no = temp_qr_msg[2]-1
    print("Found new marker no.: {}!".format(mark_no+1))

    # Adjust the marker position to the middle of the screen
    print("Aiming...")
    counter = 1
    last_turn = False
    x_error = 1
    while x_error > 0.015 and counter < 3:
      while functions.marker_is_visible() and x_error > 0.015 and counter < 3:
        if functions.marker_pose_rel().position.x > 0:
          print(functions.marker_pose_rel().position.x)
          if last_turn:
            last_turn = False
            counter += 1
          functions.adjust(0.0, -0.2/counter)
          print("Adjust - turn right")
        else:
          if not last_turn:
            last_turn = True
            counter += 1
          functions.adjust(0.0, 0.2/counter)
          print("Adjust - turn left")
        rate.sleep()
        if functions.marker_is_visible():
          x_error = abs(functions.marker_pose_rel().position.x)
          #print("New x_error: {}".format(x_error))
    
    print("Success! Code is in the middle of the screen")

    # Get information from the QR code
    markers_letter[mark_no] = temp_qr_msg[3]
    markers_2d_origin[mark_no] = temp_qr_msg[0]
    markers_2d_next[mark_no] = temp_qr_msg[1]

    # Measure the marker's position in map's cooridnates
    markers_pose_map[mark_no] = functions.transpose_pose_rel(functions.pose_current(), functions.marker_pose_rel())
    markers_2d_map[mark_no] = (markers_pose_map[mark_no].position.x, markers_pose_map[mark_no].position.y)
    
    # Estimate the pose of the marker's coordinate system in map's coordinates (needs at least 2 markers)

    if markers_letter.count(None) < 4:
      print("Estimated origin pose:\n{0}\nwith 1+-error:{1}".format(*functions.estimate_org_pose(markers_2d_origin, markers_2d_map)))

    print("Next marker in the radius: {}+-0.5".format(functions.dist2d(markers_2d_origin[mark_no],markers_2d_next[mark_no])))
  else:
    #patrol_new()
  # # Go to the marker pose
  # functions.movebase_goal_execute(markers_pose_map[mark_no])
  # print("At the marker's position")
    go_to_marker(markers_pose_map)
  rate.sleep()

# END ALL
