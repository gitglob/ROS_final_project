Do this:
* roslaunch final_project turtlebot3_world.launch
* rosrun rviz rviz
* (in rviz: Open config -> final_project/project.rviz)
* roslaunch final_project navigation_no_localization.launch
* rosrun tf static_transform_publisher 0 0 0 0 0 0 odom map 0.001
* roslaunch final_project qr_visp.launch

Then run in /scripts folder:
* python explore_area_functions.py
