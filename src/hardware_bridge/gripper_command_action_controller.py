# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Krit Chaiso.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division


__author__ = 'Krit Chaiso'
__copyright__ = 'Copyright (c) 2015 Krit Chaiso'

__license__ = 'BSD'
__maintainer__ = 'Krit Chaiso'
__email__ = 'krit.c@ku.th'


import rospy
import actionlib

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from control_msgs.msg import GripperCommand
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandFeedback
from control_msgs.msg import GripperCommandResult


class GripperCommandActionController():
    def __init__(self, controller_namespace, controllers):
        assert len(controllers) == 1
        self.update_rate = 50
        
        self.controller_namespace = controller_namespace
        self.joint_namespace = controllers[0].controller_namespace
        
    def initialize(self):
        ns = self.controller_namespace + '/gripper_command_action_node/constraints'
        self.goal_tolerance_constraint = rospy.get_param(ns + '/goal_tolerance', 0.1)
        self.effort_tolerance_constraint = rospy.get_param(ns + '/effort_tolerance', 0.1)
        self.velocity_constraint = rospy.get_param(ns + '/velocity', 0.4)
        
        # Message containing currest state of gripper
        self.msg = GripperCommandFeedback()
        self.msg.position = 0.0
        self.msg.effort = 0.0
        self.msg.stalled = False
        self.msg.reached_goal = False

        self.goal_effort = 50.0
        
        return True

    def start(self):
        self.running = True
        
        self.state_sub = rospy.Subscriber(self.joint_namespace + '/state', JointState, self.process_state)
        self.command_pub = rospy.Subscriber(self.joint_namespace + '/command', Float64, queue_size=None)

        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', GripperCommand, self.process_command)
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', GripperCommandFeedback, queue_size=None)
        self.action_server = actionlib.SimpleActionServer(self.controller_namespace + '/gripper_action',
                                                          GripperCommandAction,
                                                          execute_cb=self.process_gripper_action,
                                                          auto_start=False)
        self.action_server.start()

    def stop(self):
        self.running = False

    def process_command(self, msg):
        if self.action_server.is_active(): self.action_server.set_preempted()
        
        while self.action_server.is_active():
            rospy.sleep(0.01)
            
        self.process_gripper_command(msg)

    def process_state(self, msg):
        self.joint_state = msg
        self.update_state()

        self.msg.position = msg.current_pos
        self.msg.effort = msg.load
        self.state_pub.publish(self.msg)

    def update_state(self):
        if not self.joint_state.is_moving and self.joint_state.load >= self.goal_effort - self.effort_tolerance_constraint:
            self.msg.stalled = True
        else:
            self.msg.stalled = False

        if abs(self.joint_state.error) <= self.goal_tolerance_constraint :
            self.msg.reached_goal = True
        else:
            self.msg.reached_goal = False

    def process_gripper_action(self, goal):
        self.process_gripper_command(goal.command)

    def process_gripper_command(self, command):
        self.goal_effort = command.max_effort
        self.command_pub.publish(command.position)
            
        rospy.loginfo('Gripper start requested at %.3lf, waiting...', rospy.Time.now().to_sec())
        rate = rospy.Rate(self.update_rate)
        
        while not self.msg.stalled and not self.msg.reached_goal:
            rate.sleep()
            
        res = GripperCommandResult()
        res.position = self.msg.position
        res.effort =  self.msg.effort
        res.stalled = self.msg.stalled
        res.reached_goal = self.msg.reached_goal
        msg = 'Gripper execution successfully completed'
        rospy.loginfo(msg)
        self.action_server.set_succeeded(result=res, text=msg)
