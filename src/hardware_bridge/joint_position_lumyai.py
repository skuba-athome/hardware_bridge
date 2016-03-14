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


class JointPositionLumyai(JointPositionController):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointPositionController.__init__(self, dxl_io, controller_namespace, port_namespace)

        self.actual_encoder_resolution = rospy.get_param(self.controller_namespace + '/actual_encoder_resolution')
        self.range_radians = rospy.get_param('dynamixel/%s/%d/range_radians' % (self.port_namespace, self.motor_id))

    def pos_rad_to_raw(self, pos_rad):
        if pos_rad < self.min_angle: pos_rad = self.min_angle
        elif pos_rad > self.max_angle: pos_rad = self.max_angle
        pos_raw = self.rad_to_raw(pos_rad, self.initial_position_raw, self.flipped, self.actual_encoder_resolution/self.range_radians)
        return int(pos_raw * self.ENCODER_RESOLUTION/self.actual_encoder_resolution)

