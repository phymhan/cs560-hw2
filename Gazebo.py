#!/usr/bin/python 

import rospy
import numpy as np 
from std_msgs.msg import Char
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from hand_simulator.srv import MoveServos
from ackermann_msgs.msg import AckermannDrive
from std_srvs.srv import Empty, EmptyResponse
import tty, termios, sys, os
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import time


class Gazebo:
  def __init__(self):
    rospy.init_node('ackerman', anonymous=True)
    self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
    self.get_state = rospy.Subscriber("/gazebo/model_states", ModelState, self.callback)
    self.publisher = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
    self.position = None
    self.quart = None
    self.yaw = None
    
  def callback(self,res):
    self.position = [res.pose.position.x,res.pose.position.y,res.pose.position.z]
    self.quart = [res.orientation.x, res.orientation.y, res.orientation.z, res.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quart)
    roll = euler[0]
    pitch = euler[1]
    self.yaw = euler[2]

  def getState(self):
    '''
      Out:
        yaw: steering angle
    '''
    return self.position, self.yaw

  def setState(self,xyz,quart):
    '''
    set state in simulator 
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

  def action(self,speed,angle,duration):
    '''
    Do actions
    duration: time in [s]
    '''
    rate = rospy.Rate(100)
    begin = time.time()
    while not rospy.is_shutdown() and (time.time() - start_time < duration):
      msg = AckermannDrive()
      msg.steering_angle = angle
      msg.speed = speed
      self.publisher.publish(msg)
      rate.sleep()



    


    

