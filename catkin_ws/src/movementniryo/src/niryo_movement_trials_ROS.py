#!/usr/bin/env python


# Imports
from niryo_one_tcp_client import *
import sys
import rospy
import time
import random
import numpy as np
import math as m
import time
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from itertools import product, combinations
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int32

pose_from_camera = 0
kick_state = 0
flag_pose_goal = 0
kick_ball = 0
distance_to_ball = 1

def callback_data(data):
  global pose_from_camera
  pose_from_camera = data.data
  print("Mensagem recebida", time.ctime(time.time()))
  print("callback -> " , pose_from_camera)

def callback_kick(kick_data):
  global kick_state
  kick_state = kick_data.data

def callback_pose_goal(pose_goal_data):
  global flag_pose_goal
  flag_pose_goal = pose_goal_data.data

# compute angle 1
def compute_t1(TL_pos_world):
  res = m.atan2(TL_pos_world[1], TL_pos_world[0])

  if (TL_pos_world[0] > 0 or TL_pos_world[0] == 0):
    return res
  else:
    if (TL_pos_world[1] < 0):
      return res - m.pi
    else:
      return res + m.pi

def compute_HL_pos(L7, TL_pos_shoulder):
  return np.array([TL_pos_shoulder[0], TL_pos_shoulder[1], TL_pos_shoulder[2] + L7]).T

def compute_WL_pos(L6x, L6z, HL_pos_shoulder):
  return np.array([HL_pos_shoulder[0] + L6z, HL_pos_shoulder[1], HL_pos_shoulder[2] + L6x]).T

def compute_t2_to_t6(L2, L3, L4x, L4z, L5, WL_pos_shoulder, t1):
  x, z = WL_pos_shoulder[0], WL_pos_shoulder[2]
  
  # length of auxiliary arm between elbow link and wrist link 
  L45 = m.sqrt(L4z**2 + (L5 + L4x)**2)

  # alpha angle
  alpha = m.atan2(L4z, L5 + L4x)

  # beta angle
  beta = m.acos((x**2+(z-L2)**2-L3**2-L45**2)/(2*L3*L45))

  # gama angle
  gama = m.atan2(z-L2, x)

  # phi angle
  phi = m.acos((x**2+(z-L2)**2+L3**2-L45**2)/(2*m.sqrt(x**2+(z-L2)**2)*L3))

  # compute t2 to t6
  t2 = - m.pi/2 + gama + phi
  t3 = m.pi/2 - beta - alpha
  t4 = t1
  t5 = - t2 - t3 - alpha + 0.15 # ??? 0.15 to reach correct angle
  t6 = -t1/2
  
  return t2, t3, t4, t5, t6

def world_to_shoulder(t1, L1, pos_world):
  # transformation from base link to shoulder link
  T_BS = np.array([[m.cos(t1), m.sin(t1), 0, 0],
                   [-m.sin(t1), m.cos(t1), 0, 0],
                   [0, 0, 1, -L1],
                   [0, 0, 0, 1]])

  # apply transformation to position
  pos_shoulder = np.matmul(T_BS, cartesian_to_homogeneous(pos_world))

  return homogeneous_to_cartesian(pos_shoulder)

# transforms vector from homogeneous to cartesian coordinates
def homogeneous_to_cartesian(hmg):
  return np.delete(hmg, 3)

# transforms vector from cartesian to homogeneous coordinates
def cartesian_to_homogeneous(cart):
  return np.append(cart.T, 1).T

# compute tool link position in world referential based on joint angles
def direct_kinematics(angles, L):
  [t1, t2, t3, t4, t5, t6] = angles
  [L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = L

  # transformation from shoulder link to base link
  T_SB = np.array([[m.cos(t1), -m.sin(t1), 0, 0],
                   [m.sin(t1), m.cos(t1), 0, 0],
                   [0, 0, 1, L1],
                   [0, 0, 0, 1]])
  
  # transformation from arm link to shoulder link
  T_AS = np.array([[-m.sin(t2), -m.cos(t2), 0, 0],
                   [0, 0, -1, 0],
                   [m.cos(t2), -m.sin(t2), 0, L2],
                   [0, 0, 0, 1]])

  # transformation from elbow link to arm link
  T_EA = np.array([[m.sin(t3), m.cos(t3), 0, L3],
                   [-m.cos(t3), m.sin(t3), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

  # transformation from forearm link to elbow link
  T_FE = np.array([[0, 0, 1, L4x],
                   [m.sin(t4), m.cos(t4), 0, L4z],
                   [-m.cos(t4), m.sin(t4), 0, 0],
                   [0, 0, 0, 1]])

  # transformation from wrist link to forearm link
  T_WF = np.array([[0, 0, -1,0],
                   [m.sin(t5), m.cos(t5), 0, 0],
                   [m.cos(t5), -m.sin(t5), 0, L5],
                   [0, 0, 0, 1]])

  # transformation from hand link to wrist link
  T_HW = np.array([[0, 0, 1, L6x],
                   [m.sin(t6), m.cos(t6), 0, -L6z],
                   [-m.cos(t6), m.sin(t6), 0, 0],
                   [0, 0, 0, 1]])

  # transformation from tool link to hand link
  T_TH = np.array([[0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [1, 0, 0, L7],
                   [0, 0, 0, 1]])

  # transformation from tool link to base link
  T_TB = T_SB.dot(T_AS).dot(T_EA).dot(T_FE).dot(T_WF).dot(T_HW)

  T_03 = T_SB.dot(T_AS).dot(T_EA)

  T_36 = np.linalg.inv(T_03).dot(T_TB)

  # position of origin of tool link in tool link referential
  TL_pos = np.array([0, 0, 0]).T

  # tool link position in world/base link referential
  TL_pos_world = homogeneous_to_cartesian(np.matmul(T_TB, cartesian_to_homogeneous(TL_pos)))

  return TL_pos, TL_pos_world

def inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, TL_pos_world):
  # vector for the joint angles
  t = [0, 0, 0, 0, 0, 0]

  # compute t1
  t[0] = compute_t1(TL_pos_world)

  # transform desired position from world referential to shoulder referential
  TL_pos_shoulder = world_to_shoulder(t[0], L1, TL_pos_world)

  # compute hand link position in shoulder referential
  HL_pos_shoulder = compute_HL_pos(L7, TL_pos_shoulder)

  # compute wrist link position in shoulder referential
  WL_pos_shoulder = compute_WL_pos(L6x, L6z, HL_pos_shoulder)

  # compute angles 2, 3, 4, 5 and 6
  t[1], t[2], t[3], t[4], t[5] = compute_t2_to_t6(L2, L3, L4x, L4z, L5, WL_pos_shoulder, t[0])

  return t

if __name__ == "__main__":

  rospy.init_node("movementniryo")
  rate = rospy.Rate(20)

  rospy.Subscriber("goal_pose", Float32, callback_data)
  rospy.Subscriber("kick_state", Int32, callback_kick)
  rospy.Subscriber("flag_pose", Int32, callback_pose_goal)

  # Connecting to robot
  niryo_one_client = NiryoOneClient()
  niryo_one_client.connect("169.254.200.200")  # =< Replace by robot ip address 10.2.0.104 169.254.200.200
  #niryo_one_client.connect("127.0.0.1")

  # Trying to calibrate
  status, data = niryo_one_client.calibrate(CalibrateMode.AUTO)
  if status is False:
      print("Error: " + data)

  niryo_one_client.set_arm_max_velocity(100)

  # Getting pose
  #status, pose_array, data = niryo_one_client.get_pose()
  #initial_pose = None
  #pose = []
  #if status is True:
  #    initial_pose = data
  #    pose = pose_array
  #else:
  #    print("Error: " + data)
  #pose = [pose[0]*1000, pose[1]*1000, pose[2]*1000]
  #print pose
  #print("Posicao Inicial",pose)
  #print(initial_pose)

  while not rospy.is_shutdown():
    
    #######################INITIAL POSITION#######################
    
    pose_X_robot = 0

    goal = [150, pose_X_robot, 90]
    goal = [200, 200, 70]
    goal = np.array(goal)
    #print("Posicao Objetivo", goal)

    # Getting joints
    #status, data = niryo_one_client.get_joints()
    #initial_joints = None
    #if status is True:
    #  initial_joints = data
    #else:
    #  print("Error: " + data)
    #print("Angulos Iniciais", initial_joints)


    #INVERSE KINEMATICS
    [L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
    angles = inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, goal)
    #print("Resultado do Inverse Kinematics",angles)


    #DIRECT KINEMATICS
    L = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
    TL_pos_2, resultado2 = direct_kinematics(angles, L)
    #print("Resultado do Direct Kinematics", resultado2)

    # Move Joints
    status, data = niryo_one_client.move_joints(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
    if status is False:
      print("Error: " + data)

    #######################GOAL POSITION#######################
    #If goal position is obtained (flag)
    print("flag_pose_goal = ", flag_pose_goal)
    if flag_pose_goal == 1:
      print("flag_pose_goal = ", flag_pose_goal)
      
      while kick_state == 0:
        
        print("entrei2")

        pose_X_robot = pose_from_camera #replace with the one from camera
        print(pose_from_camera)

        goal = [150, pose_X_robot, 90]
        #goal = [200, 200, 70]
        goal = np.array(goal)
        #print("Posicao Objetivo", goal)

        # Getting joints
        #status, data = niryo_one_client.get_joints()
        #initial_joints = None
        #if status is True:
        #  initial_joints = data
        #else:
        #  print("Error: " + data)
        #print("Angulos Iniciais", initial_joints)

        print("antes de mexer", time.time())
        #INVERSE KINEMATICS
        [L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
        angles = inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, goal)
        #print("Resultado do Inverse Kinematics",angles)

        #DIRECT KINEMATICS
        L = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
        TL_pos_2, resultado2 = direct_kinematics(angles, L)
        #print("Resultado do Direct Kinematics", resultado2)
        
        before_move = time.time()

        print("Vou mexer agora",time.time())
        # Move Joints
        status, data = niryo_one_client.move_joints(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
        if status is False:
          print("Error: " + data)

        after_move = time.time()
        print("Ja mexi!",time.time())

        print("intervalo = ", after_move - before_move)


        #if kick_state == 1: ##receive this flag if the distance is close to the arm
      print("entrei 3")
          
          #######################KICK POSITION#######################
          #WHILE FLAG DE APROXIMACAO == 0, FAZER ISTO
          #ELSE, KICK
         #replace with the one from camera

      goal = [300, pose_X_robot, 120]
      #goal = [200, 200, 70]
      goal = np.array(goal)
      #print("Posicao Objetivo", goal)

      # Getting joints
      #status, data = niryo_one_client.get_joints()
      #initial_joints = None
      #if status is True:
      #  initial_joints = data
      #else:
      #  print("Error: " + data)
        #print("Angulos Iniciais", initial_joints)


      #INVERSE KINEMATICS
      [L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
      angles = inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, goal)
      #print("Resultado do Inverse Kinematics",angles)


      #DIRECT KINEMATICS
      L = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
      TL_pos_2, resultado2 = direct_kinematics(angles, L)
      #print("Resultado do Direct Kinematics", resultado2)


      # Move Joints
      status, data = niryo_one_client.move_joints(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
      if status is False:
        print("Error: " + data)

      pose_X_robot = 0
      goal = [150, pose_X_robot, 90]
      #goal = [200, 200, 70]
      goal = np.array(goal)
      #print("Posicao Objetivo", goal)

      # Getting joints
      #status, data = niryo_one_client.get_joints()
      #initial_joints = None
      #if status is True:
      #  initial_joints = data
      #else:
      #  print("Error: " + data)
      #print("Angulos Iniciais", initial_joints)


      #INVERSE KINEMATICS
      [L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
      angles = inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, goal)
      #print("Resultado do Inverse Kinematics",angles)


      #DIRECT KINEMATICS
      L = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
      TL_pos_2, resultado2 = direct_kinematics(angles, L)
      #print("Resultado do Direct Kinematics", resultado2)

      # Move Joints
      status, data = niryo_one_client.move_joints(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
      if status is False:
        print("Error: " + data)

    # Getting hardware information
    # status, data = niryo_one_client.get_digital_io_state()
    # if status is True:
    #     digital_pin_array = data
    #     for digital_pin in digital_pin_array:
    #         print("Pin: " + digital_pin.pin_id
    #               + ", name: " + digital_pin.name
    #               + ", mode: " + str(digital_pin.mode)
    #               + ", state: " + str(digital_pin.state))

    #status, data = niryo_one_client.set_learning_mode(True)
    #if status is False:
      #print("Error: " + data)
      
    #niryo_one_client.quit()

    rate.sleep()

  #Turning learning mode ON


