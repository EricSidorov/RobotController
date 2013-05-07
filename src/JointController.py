#! /usr/bin/env python
import roslib; roslib.load_manifest('RobotController')
import math, rospy, os, rosparam
from RobotController.msg import * 
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from numpy import zeros, array, linspace, arange
import numpy as np
from math import ceil
import yaml
from copy import copy



class JointCommands_msg_handler(object):
    # A class for composing and sending osrf JointCommands messages
    #
    # osrf empty message: 
    #        string[] name  - names of the joints to control (need to contain *all* atlas joints in order to work)
    #
    #     commands (order should be same as name[]):
    #        float64[] position
    #        float64[] velocity
    #        float64[] effort
    #
    #     gains (order should be same as name[]):
    #        float64[] kp_position
    #        float64[] ki_position
    #        float64[] kd_position
    #        float64[] kp_velocity
    #        float64[] i_effort_min
    #        float64[] i_effort_max
    #
    # object of this class stores one Joint command message (self._command)
    # the class implements method that allow easy command sending to individual joints while remembering previous command to the unaffected joints
    # 
    # the default command is:
    # name[28] = atlasJointNames (a list of all joints names in the drc robot)
    # position velocity and effort [28] all set to 0
    # gains set to default values taken from parameter server
    # the default parameter is '/atlas_controller/gains/' which is loaded automaticly with drcsim
    # 
    # reset_command() : resets self._command to its default gains and commands
    #
    # set_default_gains_from_param(param_name='/atlas_controller'): sets the default gains from parameter server, by default the param is /atlas_controller
    #
    # set_default_gains_from_yaml(yaml_path): sets the default gains from a yaml file, note: gains must be defined in /atlas_controller/gains/, use atlas_controller.yaml as template
    #
    # reset_gains(joints = 'all'): resets the gains of joints to default, resets all joints by default
    #
    # set_pos(joint,pos): sets a position command to the joint with default gains, joint may be specified as string (joint name) or number
    #
    # set_eff(joint,eff): sets an effort command to joint, position and velocity PID gains are set to 0, joint may be specified as string (joint name) or number
    #
    # set_gains(joint,p,i,d): sets PID gains of a joint, joint may be specified as string (joint name) or number
    #
    # send_command(): publishes the command to the controller
    #
    # print_command(): prints the command (useful for debugging)

    def __init__(self,robot_name,robot_joints):

        self.RobotName = robot_name
        self.JointNames = robot_joints
        self.Joint_dict = {self.JointNames[i]:i for i in xrange(len(self.JointNames))}
        self.inv_dict = dict(zip(self.Joint_dict.values(), self.Joint_dict.keys()))

        if rospy.get_name() == '/unnamed':
            rospy.init_node('JointCommands_msg_handler',anonymous=True)
        self._compub = rospy.Publisher('/'+self.RobotName+'/robot_command', RobotCommand)
        self._command = RobotCommand()
        self.set_default_gains_from_param()
        self.reset_command()

    def set_default_gains_from_yaml(self,yaml_path):
        yaml_file = file(yaml_path)
        self._default_gains = yaml.load(yaml_file)
        self._default_gains = self._default_gains[self.RobotName]['gains']

    def set_default_gains_from_param(self,param_name = 0):
        if param_name == 0:
            param_name = '/'+self.RobotName
        self._default_gains = rospy.get_param(param_name)
        self._default_gains = self._default_gains['gains']

    def reset_command(self):
        n = len(self.JointNames)
        self._command.position     = zeros(n)
        self._command.velocity     = zeros(n)
        self._command.effort       = zeros(n)
        self._command.kp_position  = zeros(n)
        self._command.ki_position  = zeros(n)
        self._command.kd_position  = zeros(n)
        self._command.kp_velocity  = zeros(n)
        self._command.i_effort_min = zeros(n)
        self._command.i_effort_max = zeros(n)
        self.reset_gains()

    def reset_gains(self,joints = 'ALL'):
        if joints == 'ALL':
            n = len(self.JointNames)
            lst = range(n)
        else:
            lst = list([joints])
        for i in lst:
          name = self.JointNames[i]
          self._command.kp_position[i]  = self._default_gains[name]['p']
          self._command.kd_position[i]  = self._default_gains[name]['d']
          self._command.ki_position[i]  = self._default_gains[name]['i']
          self._command.i_effort_max[i] = self._default_gains[name]['i_clamp']
          self._command.i_effort_min[i] = -self._command.i_effort_max[i]

    def set_pos(self,joint, pos):
        if type(joint) == int:
            joint_num = int(joint)
        elif type(joint) == str:
            joint_num = self.Joint_dict[joint]
        else:
            raise TypeError
            return
        name = self.JointNames[joint_num]
        self._command.position[joint_num] = float(pos)
        self.reset_gains(joint_num)

    def set_eff(self,joint,eff,null_gains = True):
        if type(joint) == int:
            joint_num = int(joint)
        elif type(joint) == str:
            joint_num = self.Joint_dict[joint]
        else:
            raise TypeError
            return
        self._command.effort[joint_num] = float(eff)
        if null_gains:
            self.set_gains(joint_num,0.0,0.0,0.0,set_default = False)

    def set_mixed(self,joint,eff,pos):
        if type(joint) == int:
            joint_num = int(joint)
        elif type(joint) == str:
            joint_num = self.Joint_dict[joint]
        else:
            raise TypeError
            return
        self._command.effort[joint_num] = float(eff)
        self._command.position[joint_num] = float(pos)

    def set_gains(self,joint,p,i,d,set_default = True):
        if type(joint) == int:
            joint_num = joint
            joint_name = self.inv_dict[joint_num]
        elif type(joint) == str:
            joint_name = joint
            joint_num = self.Joint_dict[joint]
        else:
            raise TypeError
            return
        #update defaults
        if set_default:
            self._default_gains[joint_name]['p'] = p
            self._default_gains[joint_name]['i'] = i
            self._default_gains[joint_name]['d'] = d
        #
        self._command.kp_position[joint_num]  = p
        self._command.ki_position[joint_num]  = i
        self._command.kd_position[joint_num]  = d

    def print_command(self):
        for k in xrange(len(self.JointNames)):
            print 'name: ', self._command.name[k]
            print 'command: pos:', self._command.position[k], 'eff:', self._command.effort[k]
            print 'p: ', self._command.kp_position[k]
            print 'i: ', self._command.ki_position[k]
            print 'd: ', self._command.kd_position[k]

    def send_command(self):
        self._command.header.stamp = rospy.Time.now()
        self._compub.publish(self._command)
    def get_command(self):
        return self._command

    def set_all_pos(self,pos_vec):
        if len(pos_vec) == len(self.JointNames):
            self._command.position = list(pos_vec)
        else:
            print 'position command legth doest fit'

    def send_pos_traj(self,pos1,pos2,T,dt):
        if len(pos1) == len(pos2) == len(self.JointNames):
            N = ceil(T/dt)
            pos1 = array(pos1)
            pos2 = array(pos2)
            for ratio in linspace(0, 1, N):
              interpCommand = (1-ratio)*pos1 + ratio * pos2
              self._command.position = [ float(x) for x in interpCommand ]
              self.send_command()
              rospy.sleep(dt)
        else:
            print 'position command legth doest fit'
