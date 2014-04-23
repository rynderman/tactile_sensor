#!/usr/bin/env python
import roslib
roslib.load_manifest('tactile_sensor')
import sys
import rospy
#import cv
import cv2.cv as cv

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import baxter_interface as baxter
import baxter_core_msgs.msg as baxter_msgs
import geometry_msgs.msg as geometry_msgs
import handle_detector.msg as handle_msgs
import math
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
import numpy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import time
import tf

# global variables
joint_names = []
joint_values_curr = []

class insertion_vision:

    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic_2", Image)

        cv.NamedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.image_callback)
        
        # subscribe to ROS topic about right arm joint commands
        rospy.Subscriber('robot/joint_states', sensor_msgs.JointState, self.arm_callback)
        # create publisher for right arm joint commands
        self.right_arm_pub = rospy.Publisher('/robot/limb/right/joint_command', baxter_msgs.JointCommand)
        # create publisher for gripper joint control
        self.gripper_pub = rospy.Publisher('/robot/end_effector/right_gripper/command', baxter_msgs.EndEffectorCommand)
          
        # wait for IK service, and create IK solver
        rospy.wait_for_service('/compute_ik')
        self.ik_solver = rospy.ServiceProxy('compute_ik', GetPositionIK)
          
        # create transform listener
        self.tf_listener = tf.TransformListener()
          
        # create transform broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
          
        # turn Baxter on
        baxter.RobotEnable().enable()  # @UndefinedVariable

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        #(cols, rows) = cv.GetSize(cv_image)
        cv.ShowImage("Image window", cv_image)
                
        #try:
        #    self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
        #except CvBridgeError, e:
        #    print e
    def arm_callback(self, joint_state):
        global joint_names
        global joint_values_curr
        #~ print joint_state.position
        #~ print joint_state.name
        joint_names = joint_state.name[9:16]
        joint_values_curr = list(joint_state.position[9:16])
        #~ print joint_values_curr
        #~ rospy.loginfo(rospy.get_name() + ": I heard %s" % joint_names)
  
def main():
    ic = insertion_vision()
    rospy.init_node('insertion', anonymous=True)    
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()
    # turn Baxter off
    baxter.RobotEnable().disable() # @UndefinedVariable
    return 0

if __name__ == '__main__':
    sys.exit(main())
