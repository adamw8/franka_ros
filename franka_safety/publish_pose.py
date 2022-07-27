#!/usr/bin/env python2

import rospy
import numpy as np
import yaml
import sys
from time import sleep

from geometry_msgs.msg import PoseStamped

def main(argv):
  rospy.init_node("position_publisher")
  # may need to change this to not have panda
  pub = rospy.Publisher("/panda/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=1)
  print("Sleeping for 0.1s to set up pub")
  sleep(0.1)
  
  msg = PoseStamped()
  msg.header.frame_id = "panda_link0"
  msg.pose.position.x = float(argv[1])
  msg.pose.position.y = float(argv[2])
  msg.pose.position.z = float(argv[3])
  
  # check if orientation was specified
  if len(argv) == 8:
    msg.pose.orientation.x = float(argv[4])
    msg.pose.orientation.y = float(argv[5])
    msg.pose.orientation.z = float(argv[6])
    msg.pose.orientation.w = float(argv[7])
  else:
    msg.pose.orientation.x = 1.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 0.0
  
  print(msg)
  pub.publish(msg)


if __name__ == "__main__":
  main(sys.argv)
  
