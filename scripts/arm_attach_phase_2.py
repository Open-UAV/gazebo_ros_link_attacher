#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from std_msgs.msg import String

DOCKING_DEGREE = 90

DOCKING_RANGE = 1

global mode
mode = None
class GazeboLinkPose:

  def __init__(self, link_name):
    self.link_name = link_name
    self.link_pose = PoseStamped()

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

  def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose.pose = data.pose[ind]
      self.link_pose.header.frame_id = "/base_link"
      self.link_pose.header.stamp = rospy.Time.now()

    except ValueError:
      pass


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def set_mode(msg):
    global mode
    mode = str(msg.data)

if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    iris = GazeboLinkPose('iris_1')
    probe = GazeboLinkPose('sample_probe')
    mag = 1
    degrees = 90
    rate = rospy.Rate(20)

    rospy.Subscriber('/attach', String, callback=set_mode)
    # Wait for probe and iris pose values to get updated
    while (probe.link_pose == PoseStamped() or iris.link_pose == PoseStamped()):
        pass


    while not rospy.is_shutdown():
        if mode == "ATTACH":
            print("inside attach")
            vector_iris = np.array([
                iris.link_pose.pose.position.x-probe.link_pose.pose.position.x,
                iris.link_pose.pose.position.y-probe.link_pose.pose.position.y,
                iris.link_pose.pose.position.z-probe.link_pose.pose.position.z
                ])
            mag = np.sqrt(vector_iris.dot(vector_iris))
            degrees = np.degrees(np.arccos(np.clip(vector_iris[2] / mag, -1.0, 1.0)))

                # The vector here is the position of iris from probe
                # mag shows the magnitude of the vector
                # degrees shows the angle the vector makes with z axis
                # Print below to see the magnitude and degrees
            # print("{} {}".format(mag, np.degrees(np.arccos(np.clip(vector_iris[2] / mag, -1.0, 1.0)))))

            # 15 degree threshold and magnitude is less than 0.6 meters, attach works
            if mag < DOCKING_RANGE and degrees < DOCKING_DEGREE:
                rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
                attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                                Attach)
                attach_srv.wait_for_service()
                rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

                # Link them
                rospy.loginfo("Attach drone to sample probe")
                req = AttachRequest()
                req.model_name_1 = "iris_1"
                req.link_name_1 = "base_link"
                req.model_name_2 = "sample_probe"
                req.link_name_2 = "base_link"
                attach_srv.call(req)
            mode = None
        elif mode == "DETACH":
            rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
            attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                            Attach)
            attach_srv.wait_for_service()
            rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

            # Link them
            rospy.loginfo("Detaching iris and sample_probe")
            req = AttachRequest()
            req.model_name_1 = "iris_1"
            req.link_name_1 = "base_link"
            req.model_name_2 = "sample_probe"
            req.link_name_2 = "base_link"
            attach_srv.call(req)
            mode = None
        rate.sleep()

