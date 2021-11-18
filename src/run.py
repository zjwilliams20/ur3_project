#!/usr/bin/env python
# run.py

import rospy

from chess import ChessGame

if __name__ == '__main__':

    rospy.loginfo('[run] Starting main...')

    # Initialize ROS node
    rospy.init_node('main')

    # Let's play a game
    game = ChessGame()
    game.play()

    rospy.spin()
    rospy.loginfo('[run] Done.')

