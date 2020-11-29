def go_to_marker(markers_pose_map):
	if markers_letter.count(None) == 2:
		markers_pose_map[mark_no].position.x - 7
		markers_pose_map[mark_no].position.y - 7
		functions.movebase_goal_execute(markers_pose_map[mark_no])
		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
		relative_angle = 360
		while(current_angle < relative_angle):
			if not functions.marker_is_visible():
		# Rotating 360 degrees
				functions.adjust(0.0, -(360*3.14/360))
				t1 = rospy.Time.now().to_sec()
				current_angle = angular_speed*(t1-t0)
			else:
				current_angle = relative_angle

