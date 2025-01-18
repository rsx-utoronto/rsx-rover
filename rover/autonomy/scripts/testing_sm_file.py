#!/usr/bin/python3

import rospy
import smach
import smach_ros
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import time

class IO:
    def __init__(self):
        self.sub = rospy.Subscriber("/imu/orient", Imu, self.imu_callback)
        self.pub = rospy.Publisher("/state", String, queue_size=10)
        self.imu = Imu()
        
    def imu_callback(self, msg):
        self.imu = msg
        print(self.imu.header.stamp.secs, "in imu_callback")

    def get_imu(self):
        return self.imu

    def pub_state(self, state):
        self.pub.publish(state)

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        self.io = None

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        print(self.io.get_imu().header.stamp.secs, "in foo")
        self.io.pub_state("foo")
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'
        
    def set_io(self, io: IO):
        self.io = io


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.io = None

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        print(self.io.get_imu().header.stamp.secs, "in bar")
        self.io.pub_state("bar")
        
        return 'outcome1'
        
    def set_io(self, io: IO):
        self.io = io


# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])
        self.io = None

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        print(self.io.get_imu().header.stamp.secs, "in bas")
        self.io.pub_state("bas")
        return 'outcome3'
    
    def set_io(self, io: IO):
        self.io = io



def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    io = IO()
    bas = Bas()
    foo = Foo()
    bas.set_io(io)
    foo.set_io(io)
    bar = Bar()
    bar.set_io(io)

    time.sleep(1)
    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', bas,
                               transitions={'outcome3':'SUB'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['outcome4', "outcome6"])

        # Open the container
        with sm_sub:

            # Add states to the container
            smach.StateMachine.add('FOO', foo, 
                                   transitions={'outcome1':'BAR', 
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', bar, 
                                   transitions={'outcome1':'outcome6'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome4':'outcome5',
                                            "outcome6" : "outcome5"})

    # Execute SMACH plan
    outcome = sm_top.execute()



if __name__ == '__main__':
    main()