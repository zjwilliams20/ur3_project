#!/usr/bin/env python
# run.py

import rospy

from ur3_controller import Ur3Controller, SPIN_RATE
from image_manager import ImageManager


if __name__ == '__main__':

    rospy.loginfo('[run] Starting main...')

    # Initialize ROS node
    rospy.init_node('main')

    # Kick off image manager
    img_mgr = ImageManager()

    # Kick off ur3 manager
    ur3_mgr = Ur3Controller(rospy.Rate(SPIN_RATE))
    ur3_mgr.demo()

    rospy.spin()
    rospy.loginfo('[run] Done.')

