#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from smach import CBState
from std_msgs.msg import String
from random import randint
  

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        if self.counter == 0:
            return 'outcome1'


#Rode cirkel         
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','failed'])
       
    def execute(self, userdata):
        a = []
        def callback(data):
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            
            test = data.data
            test2 = (test.split(" ")[0])
            test3 = (test.split(" ")[1])
            if (test2 == 'vierkant' and test3 == 'rood'):
                a.append('a')  
            if (test2 == 'cirkel' and test3 == 'geel'):
                a.append('a') 
                a.append('a')
            if (test2 == 'vierkant' and test3 == 'groen'):
                a.append('a')
                a.append('a')
                a.append('a')        
            if (test2 == 'vierkant' and test3 == 'geel'):
                a.append('a')
                a.append('a')
                a.append('a')
                a.append('a')
            
        def listener():
            rospy.Subscriber("detector", String, callback)

        listener()
        rate = 10
    	r = rospy.Rate(rate)
        rospy.loginfo('Executing state: move')
        velocity_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        rospy.loginfo('Speed to 0.2')
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        while not rospy.is_shutdown():
            #Loop to move the turtle
            while(1):
                velocity_publisher.publish(vel_msg)
                if (len(a) == 1):
                    return 'outcome1'
                if (len(a) == 2):
                    return 'outcome2'
                if (len(a) == 3):
                    return 'failed'
                if (len(a) == 4):
                    return 'outcome3'
                r.sleep()
            vel_msg.linear.x = 0
            #Stops the robot
            velocity_publisher.publish(vel_msg)
        
        
#Rood vierkant
class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','failed'])
    
    def execute(self, userdata): 
        a = []
        def callback(data):
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            test = data.data
            test2 = (test.split(" ")[0])
            test3 = (test.split(" ")[1])
            if (test2 == 'cirkel' and test3 == 'rood'):
                a.append('a')
            if (test2 == 'cirkel' and test3 == 'geel'):
                a.append('a')
                a.append('a')
            if (test2 == 'vierkant' and test3 == 'groen'):
                a.append('a')
                a.append('a')
                a.append('a')
            if (test2 == 'vierkant' and test3 == 'geel'):
                a.append('a')
                a.append('a')
                a.append('a')
                a.append('a')
            
        def listener():
            rospy.Subscriber("detector", String, callback)

        listener()
        rate = 10
    	r = rospy.Rate(rate)
        rospy.loginfo('Executing state: turn')
        velocity_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.5

        while not rospy.is_shutdown():
            while(1):             
                velocity_publisher.publish(vel_msg)
                if (len(a) == 1):
                    return 'outcome1'
                if (len(a) == 2):
                    return 'outcome2'
                if (len(a) == 3):
                    return 'failed'
                if (len(a) == 4):
                    return 'outcome3'
                r.sleep()

#Gele cirkel
class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3', 'failed'])
    
    def execute(self, userdata):
        a = []
        def callback(data):
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            
            test = data.data
            test2 = (test.split(" ")[0])
            test3 = (test.split(" ")[1])
            if (test2 == 'vierkant' and test3 == 'rood'):
                a.append('a')
            if (test2 == 'cirkel' and test3 == 'rood'):
                a.append('a')
                a.append('a')
            if (test2 == 'vierkant' and test3 == 'groen'):
                a.append('a')
                a.append('a')
                a.append('a')
            if (test2 == 'vierkant' and test3 == 'geel'):
                a.append('a')
                a.append('a')
                a.append('a')
                a.append('a')
            
        def listener():
            rospy.Subscriber("detector", String, callback)

        listener()
        rate = 10
    	r = rospy.Rate(rate)
        rospy.loginfo('Executing state: stop')
        velocity_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        vel_msg = Twist()
        while not rospy.is_shutdown():
            velocity_publisher.publish(vel_msg)
            while(1):
                if (len(a) == 1):
                    return 'outcome2'
                if (len(a) == 2):
                    return 'outcome1'
                if (len(a) == 3):
                    return 'failed'
                if (len(a) == 4):
                    return 'outcome3'
                r.sleep

#Geel vierkant
class RandomState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2', 'outcome3', 'failed'])
    def execute(self, userdata):
        rand = randint(0,2)
        if (rand == 0):
            return 'outcome1'
        if (rand == 1):
            return 'outcome2'
        if (rand == 2):
            return 'outcome3'
        return 'failed'

def main():
    rospy.init_node('ColorStates')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminate'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', Init(), transitions={'outcome1':'STOP','failed':'terminate',})
        smach.StateMachine.add('MOVE', Move(), transitions={'outcome1':'TURN','outcome2':'STOP','outcome3':'RANDOMSTATE','failed':'terminate'})
        smach.StateMachine.add('TURN', Turn(), transitions={'outcome1':'MOVE','outcome2':'STOP','outcome3':'RANDOMSTATE','failed':'terminate'})
        smach.StateMachine.add('STOP', Stop(), transitions={'outcome1':'MOVE','outcome2':'TURN','outcome3':'RANDOMSTATE','failed':'terminate'})
        smach.StateMachine.add('RANDOMSTATE', RandomState(), transitions={'outcome1':'MOVE','outcome2':'TURN','outcome3':'STOP','failed':'terminate'})
    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
