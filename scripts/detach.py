#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching iris and sample_probe")
    req = AttachRequest()
    req.model_name_1 = "iris"
    req.link_name_1 = "base_link"
    req.model_name_2 = "sample_probe"
    req.link_name_2 = "base_link"

    attach_srv.call(req)

