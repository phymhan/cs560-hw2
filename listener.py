#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Char, Header
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from hand_simulator.srv import MoveServos
from ackermann_msgs.msg import AckermannDrive
from std_srvs.srv import Empty, EmptyResponse
import tty, termios, sys, os
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import time


class Server:
  def __init__(self):
    rospy.init_node('listener', anonymous=True)
    self.listener = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
    self.server = rospy.Service('/gazebo/my_model_state', GetModelState, self.service)
    self.state = None
    rospy.spin()

  def callback(self,res):
    self.state = res
    self.pose = res.pose[2]
  

  def service(self,req):
    header = Header()
    pose = self.pose
    twist = Twist()
    success = True
    status_message='1'
    # res = GetModelStateResponse(header,pose,twist,success, status_message)
    # res = GetModelState()
    # res.header = header
    # res.pose = pose
    # res.twist = twist
    # res.success = success
    # res.status_message = "1"
    
    return header,pose,twist,success, status_message


if __name__=="__main__":
  server = Server()
  
  