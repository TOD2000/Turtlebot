#!/usr/bin/python
"""
Gets the position of the balloon and commands to turtlebot to steer
Taken from

minor modifications by team turtlebot:
If statement added as the turtlebot struggled to position itself correctly to allign with the balloon
****additional statement added to spin the turtlebot if nothing is detected

Subscribes to
    /leatherback/balloon/point_balloon

Publishes commands to
    /leatherback/cmd_vel
"""
import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

K_LAT_DIST_TO_STEER     = 2.0

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class ChaseBalloon():
    def __init__(self):

        self.balloon_x         = 0.0
        self.balloon_y         = 0.0
        self._time_detected = 0.0

        self.sub_center = rospy.Subscriber("/leatherback/balloon/point_balloon", Point, self.update_balloon)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/leatherback/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")

        self._message = Twist()

        self._time_steer        = 0
        self._steer_sign_prev   = 0

    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)

    def update_balloon(self, message):
        self.balloon_x = message.x
        self.balloon_y = message.y

        self._time_detected = time.time()
        # rospy.loginfo("Balloon detected: %.1f  %.1f "%(self.balloon_x, self.balloon_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command

        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action    = 0.0
        throttle_action = 0.0

        if self.is_detected:
           
            #--- Apply steering, proportional to how close is the object
            steer_action   =-K_LAT_DIST_TO_STEER*self.balloon_x
            steer_action   = saturate(steer_action, -1, 1)
            rospy.loginfo("Steering command %.2f"%steer_action)
            if (steer_action >= 0.3) or (steer_action <= -0.3):
                throttle_action = 0.2
            else:
                throttle_action = 0.5
            
        else:
            # team turtlebot addition - slowly spin the turtlebot to look for detections
           # time.sleep(1) #for chase actions comment out this sleep
            steer_action = 0.4
           # throttle_action = 0.0            

        return (steer_action, throttle_action)

        


    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #-- Get the control action
            steer_action, throttle_action    = self.get_control_action()

            rospy.loginfo("Steering = %3.1f"%(steer_action))

            #-- update the message
            self._message.linear.x  = throttle_action
            self._message.angular.z = steer_action

            #-- publish it
            self.pub_twist.publish(self._message)
            time.sleep(1)

            rate.sleep()

if __name__ == "__main__":

    rospy.init_node('chase_balloon')

    chase_balloon     = ChaseBalloon()
    chase_balloon.run()
