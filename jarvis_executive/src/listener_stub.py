#!/usr/bin/env python
# license from conrell university CS 6751

import roslib; roslib.load_manifest('jarvis_executive')
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent
from jarvis_planner.msg import PlanStatus

# all the counters go here!
UI_COUNTER = 0
HI_COUNTER = 0
JP_COUNTER = 0
a = False

# all the callbacks go here!
def user_interface_callback(userdata):
    if userdata.id >= '100':
    	print 'UI_COUNTER = 1'
    	UI_COUNTER = 1
    else:
    	print 'user didnt reach 100 yet'
    

def human_intent_callback(userdata):
    if userdata.intent == 1:
    	print 'HI cool'
	HI_COUNTER = 1
    else:
    	HI_COUNTER = 2

def jarvis_planner_callback(userdata):
    if userdata.PlanStatus == True:
    	print 'plan ready!'
    	JP_COUNTER = 1
    else:
    	JP_COUNTER = 0

# all the states go here!
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        
        while not rospy.is_shutdown() and (a==True):
            rospy.Subscriber('robot_cmd_trial', GoalID, user_interface_callback)
            rospy.Subscriber('intents_trial', Intent, human_intent_callback)
            rospy.Subscriber('PlanStatus_trial', PlanStatus, jarvis_planner_callback)
    	
    	    a = (UI_COUNTER==1) and (HI_COUNTER==1) and (JP_COUNTER==1):
    	print 'gonna shift to outcome4!!!!!!'
    	return 'outcome1' 
    
   
# main goes here!
def main():
    rospy.init_node('listener_stub', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'outcome4'})
       
    # Execute SMACH plan
    outcome = sm.execute()
    
        
if __name__ == '__main__':
    main()
    
    
    
'''
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from human_intent.msg import Intent
from jarvis_planner.msg import PlanStatus


def user_interface_callback(userdata):
    if userdata.id== '10':
      print 'correct!!!'
    else:
      print 'go away..'

def human_intent_callback(userdata):
    if userdata.intent == 1:
        print 'intent Hold!'
    else:
	    print 'intent Adjust!'

def jarvis_planner_callback(userdata):
    if userdata.PlanStatus == True:
    	print 'plan ready!'
    else:
    	print 'wait for planning...'

def listener():

    rospy.init_node('listener_stub', anonymous=True)

    rospy.Subscriber('robot_cmd_trial', GoalID, user_interface_callback)
    rospy.Subscriber('intents_trial', Intent, human_intent_callback)
    rospy.Subscriber('PlanStatus_trial', PlanStatus, jarvis_planner_callback)
    
    rospy.spin()
        
if __name__ == '__main__':
    listener()
'''
