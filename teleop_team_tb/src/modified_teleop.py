#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #team turtlebot edit
from std_msgs.msg import Float32
import sys, select, os
import time
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

####TEAM TURTLEBOT ADDITION ######
bashCommand = "rqt_image_view image:='raspicam_node/image' image_transport:=compressed &"
os.system(bashCommand)
  

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84


WAFFLE_MAX_LIN_VEL = 0
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

####TEAM TURTLEBOT ADDITION ###### 

distance_ahead = Float32()

def callback(msg): #team turtlebot addition set to limit velocity when close to obstacles
    global distance_ahead #Using global is not the best way to do things but due to lack of femiliarity with python this option works for now!
    distance_ahead = round((msg.ranges[0]),2)    
    global WAFFLE_MAX_LIN_VEL
    WAFFLE_MAX_LIN_VEL = 0.1
    if msg.ranges[0] > 0.5:
        WAFFLE_MAX_LIN_VEL = 0.26
    elif (0.3 < msg.ranges[0] <= 0.8):
        WAFFLE_MAX_LIN_VEL = 0.04
    elif (0.0 < msg.ranges[0] <= 0.3):
        WAFFLE_MAX_LIN_VEL = 0.01

    pub_stuff.publish(WAFFLE_MAX_LIN_VEL)
   
sub = rospy.Subscriber('scan', LaserScan, callback, queue_size=10)
time.sleep(5)
pub_stuff = rospy.Publisher('range_ahead', Float32, queue_size=10)




linear_vel_limit = 0 

def team_tb_publisher(msg): #team turtlebot addition set to limit velocity when close to obstacles
    global linear_vel_limit
    my_message = msg.data
    linear_vel_limit = round(my_message,2)
sub_test = rospy.Subscriber('range_ahead', Float32, team_tb_publisher, queue_size=10) 
time.sleep(1)



def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key




def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
 
def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input



def checkLinearLimitVelocity(vel):

    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel



def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel



if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    turtlebot3_model = rospy.get_param("model", "burger")
    time.sleep(1.1) # delay here is to give a chance for the publisher to catch up
    my_last_max_linear_vel = linear_vel_limit
    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
                print("distance ahead, ", + distance_ahead)             
                
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
                print("distance ahead, ", + distance_ahead)
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
                print("distance ahead, ", + distance_ahead)
            elif my_last_max_linear_vel != linear_vel_limit and distance_ahead != 0.0: ####TEAM TURTLEBOT ADDITION ###### 
                my_last_max_linear_vel = linear_vel_limit
                print("slowed down as obstacle ahead")
                print("max_ispeed is now ", + linear_vel_limit)
                print("distance ahead is ", + distance_ahead)
            else:
                
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            
            ####TEAM TURTLEBOT ADDITION TO LIMIT VELOCITY IN TELEOP  ###### 
            if 0.3 <= distance_ahead <= 0.6: 
                if control_linear_vel > 0.02:
                    target_linear_vel   = 0.02
                    control_linear_vel = 0.02
                    print("slowed down as obstacle ahead")
                    print("max_speed is now ", + linear_vel_limit)
                    print("distance ahead is ", + distance_ahead)

            elif 0.0 < distance_ahead < 0.3:
                if control_linear_vel > 0.01:
                    target_linear_vel   = 0.01
                    control_linear_vel = 0.01
                    print("slowed down as obstacle ahead")
                    print("max_speed is now ", + linear_vel_limit)
                    print("distance ahead is ", + distance_ahead)



            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
