#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Char
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from hand_simulator.srv import MoveServos
from std_srvs.srv import Empty, EmptyResponse
import tty, termios, sys, os
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import time


class Gazebo:
  def __init__(self):
    rospy.init_node('ackerman', anonymous=True)
    self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
    self.get_state_srv = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)


  def getState(self):
    get_state = GetModelState()
    get_state.model_name = "ackermann_vehicle"
    res = self.get_state_srv(get_state)
    xyz = [res.pose.position.x,res.pose.position.y,res.pose.position.z]
    quart = [res.orientation.x, res.orientation.y, res.orientation.z, res.orientation.w]
    return xyz, quart

  def setState(self,xyz,quart):
    '''
      xyz: [x,y,z]
      quart: [x,y,z,w]
    '''
    model_state = ModelState()
    model_state.model_name = "ackermann_vehicle"
    pose = Pose()
    pose.position.x = xyz[0]
    pose.position.y = xyz[1]
    pose.position.z = xyz[2]
    pose.orientation.x = quart[0]
    pose.orientation.y = quart[1]
    pose.orientation.z = quart[2]
    pose.orientation.w = quart[3]
    model_state.pose = pose
    res = self.set_state_srv.call(model_state)
    success = res.success
    return success



    


    

