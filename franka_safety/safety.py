#!/usr/bin/env python2

import rospy
import numpy as np
import yaml
from franka_msgs.srv import TriggerError
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
import time

class SafetyLayer:
  def __init__(self, robot_state_topic, ball_position_topic, safety_params):
    # setup service client request for trigger_error
    print("Waiting to set up trigger_error service")
    rospy.wait_for_service('/franka_control/trigger_error')
    self.trigger_error = rospy.ServiceProxy('/franka_control/trigger_error', TriggerError)
    print("Finished trigger setup")

    self.robot_state_topic = robot_state_topic
    self.franka_subscriber = rospy.Subscriber(robot_state_topic, FrankaState, 
      self.franka_safety_callback, tcp_nodelay=True)
    self.ball_subscriber = rospy.Subscriber(ball_position_topic, Float64MultiArray, 
      self.ball_safety_callback, tcp_nodelay=True)

    # ensure safety bounds are reasonable
    # hard limits provided by franka
    hard_lower_position_limits = {
      "joint1": -2.8973,
      "joint2": -1.7628,
      "joint3": -2.8973,
      "joint4": -3.0718,
      "joint5":-2.8973,
      "joint6":-0.0175,
      "joint7":-2.8973}
    hard_upper_position_limits = {
      "joint1": 2.8973, 
      "joint2": 1.7628, 
      "joint3": 2.8973, 
      "joint4": -0.0698, 
      "joint5": 2.8973, 
      "joint6": 3.7525, 
      "joint7": 2.8973}
    hard_velocity_limits = {
      "joint1": 2.1750, 
      "joint2": 2.1750, 
      "joint3": 2.1750, 
      "joint4": 2.1750, 
      "joint5": 2.6100, 
      "joint6": 2.6100, 
      "joint7": 2.6100}
    hard_torque_limits = {
      "joint1": 87, 
      "joint2": 87, 
      "joint3": 87, 
      "joint4": 87, 
      "joint5": 12, 
      "joint6": 12, 
      "joint7": 12}

    # check that position limits are in range
    self.joint_position_limits = safety_params["joint_position_limits"]
    for joint, limit in self.joint_position_limits.items():
      if limit["lower"] < hard_lower_position_limits[joint] or limit["lower"] > hard_upper_position_limits[joint]:
        rospy.logwarn(
          "{}'s lower position limit of {} is out of range. Using a lower position limit of {} (0.9 * hard limit) for {} instead.".format(
            joint, limit["lower"], 0.9*hard_lower_position_limits[joint], joint))
        self.joint_position_limits[joint]["lower"] = 0.9*hard_lower_position_limits[joint]
      if limit["upper"] < hard_lower_position_limits[joint] or limit["upper"] > hard_upper_position_limits[joint]:
        rospy.logwarn(
          "{}'s upper position limit of {} is out of range. Using a upper position limit of {} (0.9 * hard limit) for {} instead.".format(
            joint, limit["upper"], 0.9*hard_upper_position_limits[joint], joint))
        self.joint_position_limits[joint]["upper"] = 0.9*hard_upper_position_limits[joint]

    # check that velocity limits are in range
    self.joint_velocity_limits = safety_params["joint_velocity_limits"]
    for joint, limit in self.joint_velocity_limits.items():
      if limit > hard_velocity_limits[joint]:
        rospy.logwarn(
          "{}'s velocity limit of {} is out of range. Using a velocity limit of {} (0.9 * hard limit) for {} instead.".format(
            joint, limit, 0.9*hard_velocity_limits[joint], joint))
        self.joint_velocity_limits[joint] = 0.9*hard_velocity_limits[joint]
    
    # check that torque limits are in range
    self.joint_torque_limits = safety_params["joint_torque_limits"]
    for joint, limit in self.joint_torque_limits.items():
      if limit > hard_torque_limits[joint]:
        rospy.logwarn(
          "{}'s torque limit of {} is out of range. Using a torque limit of {} (0.9 * hard limit) for {} instead.".format(
            joint, limit, 0.9*hard_torque_limits[joint], joint))
        self.joint_torque_limits[joint] = 0.9*hard_torque_limits[joint]

    self.franka_virtual_walls = safety_params["franka_virtual_walls"]
    self.ball_virtual_walls = safety_params["ball_virtual_walls"]
    self.end_effector_limits = safety_params["end_effector_limits"]
    self.collision_limits = safety_params["collision_limits"]
    self.ball_velocity_limit = safety_params["ball_velocity_limit"]

    # member functions for computations
    self.prev_time_franka = None
    self.prev_EE_position = None
    self.prev_time_ball = None
    self.prev_ball_position = None
    self.prev_ball_velocity = 0
    self.last_ball_msg_timestamp = None

    print("Finished initializing SafetyLayer object")

  def franka_safety_callback(self, msg):
    # parse msg
    EEx = msg.O_T_EE[12] # x
    EEy = msg.O_T_EE[13] # y
    EEz = msg.O_T_EE[14] # z
    position = np.array([EEx, EEy, EEz])
    # print(position)
    # print("EE", position)

    z_axis = np.array([msg.O_T_EE[8], msg.O_T_EE[9], msg.O_T_EE[10]])
    zd = np.array([0, 0, -1])

    # check virtual walls
    for name, wall in self.franka_virtual_walls.items():
      a, b, c, d = wall['a'], wall['b'], wall['c'], wall['d']
      if ((a * EEx + b * EEy + c * EEz) <= d):
        rospy.logerr('FRANKA VIOLATED VIRTUAL WALL: ' + name)
        self.trigger_error_wrapper()

    # check end effector velocity
    t = time.time()
    if self.prev_EE_position is not None:
      velocity = (position - self.prev_EE_position) / (t-self.prev_time_franka)
      if np.linalg.norm(velocity) > self.end_effector_limits["velocity_limit"]:
        rospy.logerr("VIOLATED END EFFECTOR VELOCITY LIMIT")
        self.trigger_error_wrapper()
    
    self.prev_EE_position = position
    self.prev_time_franka = t

    # check orientation
    angle_from_vertical = np.arccos(zd.dot(z_axis))
    if angle_from_vertical > self.end_effector_limits["angle_limit"]:
      rospy.logerr("VIOLATED EE ANGLE LIMIT")
      self.trigger_error_wrapper()

    # check joint position, velocity, torque limits, collisions
    for i in range(7):
      joint = "joint{}".format(i+1)
      if msg.q[i] < self.joint_position_limits[joint]["lower"]:
        rospy.logerr(joint + ' HAS VIOLATED ITS LOWER POSITION LIMIT')
        self.trigger_error_wrapper()
      if msg.q[i] > self.joint_position_limits[joint]["upper"]:
        rospy.logerr(joint + ' HAS VIOLATED ITS UPPER POSITION LIMIT')
        self.trigger_error_wrapper()
      if abs(msg.dq[i]) > self.joint_velocity_limits[joint]:
        rospy.logerr(joint + ' HAS VIOLATED ITS VELOCITY LIMIT')
        self.trigger_error_wrapper()
      if (abs(msg.tau_J[i])) > self.joint_torque_limits[joint]:
        rospy.logerr(joint + ' HAS VIOLATED ITS TORQUE LIMIT')
        self.trigger_error_wrapper()
      if (abs(msg.tau_ext_hat_filtered[i]) > self.collision_limits[joint]):
        rospy.logerr(joint + ' HAS SENSED A COLLISION')
        self.trigger_error_wrapper()

  def ball_safety_callback(self, msg):
    # parse msg
    position = np.array([msg.data[0], msg.data[1], msg.data[2]])
    if np.isnan(position).any():
      rospy.logerr('CAMERAS DID NOT SEE BALL, RETURNED NAN POSITION')
      self.trigger_error_wrapper()

    # check virtual walls
    for name, wall in self.ball_virtual_walls.items():
      a, b, c, d = wall['a'], wall['b'], wall['c'], wall['d']
      if ((a * position[0] + b * position[1] + c * position[2]) <= d):
        rospy.logerr('BALL VIOLATED VIRTUAL WALL: ' + name)
        self.trigger_error_wrapper()
    
    t = time.time()
    if self.last_ball_msg_timestamp is not None:
      if t - self.last_ball_msg_timestamp > 0.1:
        rospy.logerr('VIOLATED BALL MESSAGE TIMEOUT')
        self.trigger_error_wrapper()
    self.last_ball_msg_timestamp = t
    

    

    # check ball velocity
    # t = time.time()
    # if self.prev_ball_position is not None:
    #   v = (position - self.prev_ball_position) / (t-self.prev_time_ball)
    #   filtered_v = 0.9*v + 0.1*self.prev_ball_velocity
    #   self.prev_ball_velocity = v

    #   if np.linalg.norm(filtered_v) > self.ball_velocity_limit:
    #     rospy.logerr('BALL VIOLATED VELOCITY LIMIT')
    #     self.trigger_error_wrapper()

    # self.prev_ball_position = position
    # self.prev_time_ball = t

  def trigger_error_wrapper(self):
    try:
        resp = self.trigger_error(True)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

if __name__ == "__main__":
  rospy.init_node("safety_script")
  param_path = "/home/dair-manipulation/adam_ws/franka_ws/catkin_ws/src/franka_ros/franka_safety/parameters_safety.yaml"
  franka_state_topic = "/franka_state_controller/franka_states"
  ball_state_topic = "/c3/position_estimate"

  with open(param_path, 'r') as stream:
      safety_params = yaml.safe_load(stream)
  print("Finished reading safety params")
  safety = SafetyLayer(franka_state_topic, ball_state_topic, safety_params)

  print("Spinning")
  rospy.spin()