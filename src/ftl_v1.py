#!/usr/bin/env python3

from Node import Node
import rospy

if __name__ == '__main__':
    try:
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


