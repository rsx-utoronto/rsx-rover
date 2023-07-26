#!/usr/bin/env python3
import rospy
import math
import geometry_msgs.msg
import tf_conversions
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64


def anglePosition():
    '''
    Prompts the user to enter 6 angles in degrees and returns an array of size 6. 
    '''
    angle1, angle2, angle3, angle4, angle5, angle6 = input(
        "Enter 6 angles (seperated by spaces) in degrees: ").split()  # Angles in degrees
    angleArray = list(map(float, [angle1, angle2, angle3, angle4, angle5, angle6]))
    for i in range(len(angleArray)):
        angleArray[i] = angleArray[i] * math.pi /180  # Conversion from degrees to radians
    return angleArray


def framePosition():
    '''
    Prompts the user to enter 3 distances and returns an array of size 3. 
    '''
    roll, pitch, yaw, x, y, z = input("Enter roll, pitch, yaw, x, y, z (seperated by spaces): ").split(
    )  # Distances from base_link in unit squares values
    tempArray = list(map(float, [roll, pitch, yaw, x, y, z]))
    posArray = [tempArray[0], tempArray[1], tempArray[2], [tempArray[3], tempArray[4], tempArray[5]]]
    return posArray


def startJointPublisher():
    '''
    Initiates arm_sim_control, uses rospy.Publisher to take in the joint states and returns jointPublisher
    '''
    # rospy.init_node("arm_sim_control")  # start node
    # refrence to the output topic, you can have multiple in a script
    jointPublisher = rospy.Publisher("joint_states", JointState, queue_size=10)
    return jointPublisher

def startGazeboJointControllers(numJoints):
    ''' Starts to publishers for the joints in gazebo

    Parameters
    ----------
    numJoints
        the number of joints in the gazebo model
    '''

    gazeboPublisher = list()

    for i in range(numJoints):
        tempController = rospy.Publisher(f"/arm/joint{i}_position_controller/command", Float64, queue_size=10)
        gazeboPublisher.append(tempController)
    
    return gazeboPublisher

def runNewJointState(jointPublisherData):
    '''
    Publishes the newJointState header, stamp, name, and position in a continuous while loop. 
    '''
    rate = rospy.Rate(10)  # 10 hz
    while not rospy.is_shutdown():
        # data to be published
        # newJointState = JointState()
        # newJointState.header = Header()
        # newJointState.header.stamp = rospy.Time.now()
        # newJointState.name = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"] # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
        # newJointState.position = anglePosition()
        # jointPublisherData.publish(newJointState)  # Send data to be published
        positionArray = framePosition() # Display End Effector position with a transform
        displayEndEffectorTransform(positionArray)
        rate.sleep()  # Controls loop rate based on what is set in the rate variable


def displayEndEffectorTransform(endEffectorPosition, referenceLink="base_link", quaternionAngles=None):
    '''
    Publishes a tf2 transform at the End Effector Position 
    '''
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = referenceLink
    t.child_frame_id = "target_position"

    posArray = endEffectorPosition
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = posArray[3][0]
    t.transform.translation.y = posArray[3][1]
    t.transform.translation.z = posArray[3][2]
    
    q = tf_conversions.transformations.quaternion_from_euler(posArray[0], posArray[1], posArray[2], 'sxyz')
    if quaternionAngles != None:
        q = quaternionAngles
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
    else:
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
    br.sendTransform(t)

def getFrameTransform(referenceFrame: str, targetFrame: str):
    ''' Gets the transform of a tf frame to another

    Gets the transform from referenceFrame to targetFrame. In other words,
    the transformation of targetFrame to referenceFrame.

    Paramters
    ---------
    referenceFrame
        the name of the frame you are basing the transformation on (the targets transform is given relative to this frame)
    targetFrame
        the name of target

    Returns
    -------
    targetTransform
        a transform message with a .translation and a .rotation component, is none if target doesn't exist
    '''
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    try:
        tfBuffer.can_transform(referenceFrame, targetFrame, rospy.Time(), rospy.Duration(0.1))
        trans = tfBuffer.lookup_transform(referenceFrame, targetFrame, rospy.Time())
        return trans.transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        return None
    except Exception as ex:
        print(ex)

def runNewJointState2(jointPublisherData, angles):
    '''
    Publishes the newJointState header, stamp, name, and position in a continuous while loop. 
    '''

    # data to be published
    newJointState = JointState()
    newJointState.header = Header()
    newJointState.header.stamp = rospy.Time.now()
    newJointState.name = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_7", "Joint_8", "Joint_9"]
    # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
    newJointState.position = angles
    jointPublisherData.publish(newJointState)  # send data to be published
    # positionArray = framePosition() # Display End Effector position with a transform
    # displayEndEffectorTransform(positionArray)

def runNewJointState3(jointPublisherData, angles):
    '''
    Publishes the newJointState header, stamp, and name for the real arm values 
    '''

    # data to be published
    newJointState = JointState()
    newJointState.header = Header()
    newJointState.header.stamp = rospy.Time.now()
    newJointState.name = ["Real_Joint_1", "Real_Joint_2", "Real_Joint_3", "Real_Joint_4", "Real_Joint_5", 
                          "Real_Joint_6", "Real_Joint_7", "Real_Joint_8", "Real_Joint_9"]
    # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
    newJointState.position = angles
    jointPublisherData.publish(newJointState)  # send data to be published



def moveInGazebo(jointControllerPublishers, angles):
    ''' Moves arm in Gazebo based on IK angles
    
    Paramters
    ---------
    jointControllerPublishers
        list of publishers for each joint controller
    angles
        list of joint angle in same order as publishers
    '''
    
    for i in range(len(jointControllerPublishers)):
        jointControllerPublishers[i].publish(angles[i-1])

if __name__ == "__main__":  # if used rosrun on this script then ...
    try:
        rospy.init_node("arm_sim_control")  # start node
        jointPublisher = startJointPublisher()
        runNewJointState(jointPublisher)

    except Exception as ex:
        rospy.loginfo("Some Error Has Occured")
        print(ex)
