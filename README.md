# Turtlebot
#Group_Turtlebot_Project. Files used to spin a turtlebot, detect a balloon then drive towards the balloon and pop it then slowly spin looking for new balloons. The code is based on code from https://github.com/tizianofiorenzani/ros_tutorials and the official turtlebot code. The work is for a university project to create a fun and engaging set of activites for childrenusing turtlebots. It is entirely non commercial.

teleop_team_tb is using the turtlebot teleop code but adds an addition to subscribe to the scan topic to determine distance in front of the robot and slow it down if there is an obstacle ahead.

balloon_detect is a slimed down version of the code used to chase a golf ball created in https://github.com/tizianofiorenzani/ros_tutorials - THANKYOU - we have learned a lot from you.  There are more additions to this  code primarily aimed at making the balloon detection process more informative.  The code is commented but the short version is use balloon_detection/include/balloon_range_thrshold.py to get your HSV values.  Edit these values in /balloon_detection/src/find_balloon.py (lines 141 and 142)

balloon_detect_loggerhead and balloon_detect_leatherback are hardcoded verisons of balloon_detect so that you can use different HSV values for 2 different robots.
