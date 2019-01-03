#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from smach import CBState
from std_msgs.msg import String
        

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.counter = 0

    def execute(self, userdata):
        #logica moet nog komen van open cv
        rospy.loginfo('Executing state Init')
        if self.counter == 0:
            return 'outcome1'
        
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','failed'])
       
    def execute(self, userdata):
        a = []
        def callback(data):
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            test = data.data
            test2 = (test.split(" ")[0])
            test3 = (test.split(" ")[1])
            if (test2 == 'cirkel' and test3 == 'rood'):
                a.append('a')
            
        
        

                     

        def listener():

            # In ROS, nodes are uniquely named. If two nodes with the same
            # name are launched, the previous one is kicked off. The
            # anonymous=True flag means that rospy will choose a unique
            # name for our 'listener' node so that multiple listeners can
            # run simultaneously.

            rospy.Subscriber("detector", String, callback)

            # spin() simply keeps python from exiting until this node is stopped

        
        rospy.loginfo('Executing state move')
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = 2
        rospy.loginfo('Speed to 2')
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        while not rospy.is_shutdown():
   
            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0
   
            #Loop to move the turtle in an specified distance
            while(current_distance > -1):             
                #Publish the velocity
                velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= 2*(t1-t0)
                if (len(a) == 1):
                    return 'outcome1'

                listener()

            #After the loop, stops the robot
            vel_msg.linear.x = 0
            #Force the robot to stop
            velocity_publisher.publish(vel_msg)
            

        
        

class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata): 

        a = []
        def callback(data):
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            test = data.data
            test2 = (test.split(" ")[0])
            test3 = (test.split(" ")[1])
            if (test2 == 'driehoek' and test3 == 'groen'):
                a.append('a')
            
        
        

                     

        def listener():

            # In ROS, nodes are uniquely named. If two nodes with the same
            # name are launched, the previous one is kicked off. The
            # anonymous=True flag means that rospy will choose a unique
            # name for our 'listener' node so that multiple listeners can
            # run simultaneously.

            rospy.Subscriber("detector", String, callback)

            # spin() simply keeps python from exiting until this node is stopped




        rospy.loginfo('Executing state turn')
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.5

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while not rospy.is_shutdown():
   
            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
   
            #Loop to move the turtle in an specified distance
            while(current_angle > -1):             
                #Publish the velocity
                velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_angle= 0.5*(t1-t0)
                if (len(a) == 1):
                    return 'outcome1'

                listener()

            #After the loop, stops the robot
            vel_msg.linear.z = 0
            #Force the robot to stop
            velocity_publisher.publish(vel_msg)

        





        return 'outcome1'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state move')
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        vel_msg.linear.x = 1
        rospy.loginfo('Speed to 1')
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        while not rospy.is_shutdown():
   
            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0
   
            #Loop to move the turtle in an specified distance
            while(current_distance < 1):             
                #Publish the velocity
                velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= 1*(t1-t0)

            #After the loop, stops the robot
            vel_msg.linear.x = 0
            #Force the robot to stop
            velocity_publisher.publish(vel_msg)
            return 'outcome1'



def main():
    rospy.init_node('SmachTest')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminate'])

    # Open the container
    with sm:
        # Add states to the container
        # Terminate conditie toevoegen
        smach.StateMachine.add('INIT', Init(), transitions={'outcome1':'MOVE'})
        smach.StateMachine.add('MOVE', Move(), transitions={'outcome1':'TURN','failed':'terminate'})
        smach.StateMachine.add('TURN', Turn(), transitions={'outcome1':'STOP'})
        smach.StateMachine.add('STOP', Stop(), transitions={'outcome1':'terminate'})
        

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()

     



