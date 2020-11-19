#!/usr/bin/env python
# BEGIN ALL
import rospy
import functions

rospy.on_shutdown(functions.stop) # stop() will be called on shutdown
g_range_ahead = 1 # anything to start

rospy.init_node('explore_area_functions')
rate = rospy.Rate(60)

while not rospy.is_shutdown():
  g_range_ahead = functions.distance()
  print g_range_ahead

  if g_range_ahead < 0.8:
    functions.move(0.0, 0.4)
    print "Turn"
  else:
    functions.move(0.4, 0.0)
    print "Drive"
  rate.sleep()

# END ALL