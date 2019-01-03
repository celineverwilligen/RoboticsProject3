#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi



def GoForward():
    # How fast will we update the movement?
    rate = 10
    r = rospy.Rate(rate)
        
    
   

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    vel_msg.linear.x = 2

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

        #Loop to move the turtle in an specified distance
    while(current_distance < 1):
        #Publish the velocity
        pub.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= 2*(t1-t0)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    pub.publish(vel_msg)
    r.sleep()



def TurnTurtle():
   
    # How fast will we update the movement?
    rate = 10
    r = rospy.Rate(rate)
        
    # Set angular speed  
    angular_speed = 0.3 #0.3
     
    # Duration and ticks corresponding to 90 degrees
    angular_duration =  (pi / 2.0) / angular_speed
    ticks = int(angular_duration * rate)

    # Initialize the movement command and set rotation speed
    vel_msg = Twist()
    vel_msg.angular.z = angular_speed  

    # Now rotate left 90 degrees
    for t in range(ticks):           
        pub.publish(vel_msg)
        r.sleep()
        
  


if __name__ == '__main__':

    rospy.init_node('TurnTurtle', anonymous=False)
    # Publisher to control Turtle's speed
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10) 
    vel_msg=Twist()
    
    try:
        i = 0
        while (i < 4):
            TurnTurtle()
            GoForward()
            i= i+1

    except:
        rospy.loginfo("MoveSquare node terminated")
            
