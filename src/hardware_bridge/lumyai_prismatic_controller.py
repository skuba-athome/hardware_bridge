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
__credits__ = 'Cara Slutter & Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Krit Chaiso'
__email__ = 'krit.c@ku.th'

import rospy
from dynamixel_controllers.joint_position_controller import JointPositionController


class LumyaiPrismaticController(JointPositionController):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointPositionController.__init__(self, dxl_io, controller_namespace, port_namespace)
        self.VOLTAGE_DROP_BIAS = 2360
        self.TUNNING_BIAS = 0

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        raw = raw + self.VOLTAGE_DROP_BIAS + self.TUNNING_BIAS #raw bias value to fix voltage drop in sensor line
        inverse_distance = 1.88170825382235e-6 * raw - 0.0040422542 
        distance = (1.0 / (inverse_distance)) - 15.6
        return (distance/100.00)
        
    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                self.joint_state.motor_temps = [state.temperature]
                self.joint_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.error = self.joint_state.goal_pos - self.joint_state.current_pos
                self.joint_state.velocity = state.speed 
                self.joint_state.load = state.load
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                
                self.joint_state_pub.publish(self.joint_state)

    def pos_rad_to_raw(self, pos_rad):
        pos_rad = pos_rad * 100 + 15.6
        inverse_distance = (1.0 / pos_rad)
        raw = 529095.560329262 * inverse_distance + 2233.4055946581 - self.VOLTAGE_DROP_BIAS - self.TUNNING_BIAS
        if raw < self.min_angle_raw : raw = self.min_angle_raw
        elif raw > self.max_angle_raw : raw = self.max_angle_raw
        return int(raw)

