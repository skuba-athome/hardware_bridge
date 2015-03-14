#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float64

from enum import Enum


class Button(Enum):
    # buttons
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    Black = 6
    Start = 7
    Center = 8


class ButtonHandler(object):
    def __init__(self, joy=None):
        self.joy = joy

    def is_active(self, button):
        return self.joy.buttons[button] == 1

    def a_active(self):
        return self.is_active(Button.A)

    def b_active(self):
        return self.is_active(Button.B)

    def x_active(self):
        return self.is_active(Button.X)

    def y_active(self):
        return self.is_active(Button.Y)

    def lb_active(self):
        return self.is_active(Button.LB)

    def rb_active(self):
        return self.is_active(Button.RB)

    def black_active(self):
        return self.is_active(Button.Black)

    def start_active(self):
        return self.is_active(Button.Start)

    def center_active(self):
        return self.is_active(Button.Center)

    def arrow_up_active(self):
        return self.joy.axes[7] == 1.0

    def arrow_down_active(self):
        return self.joy.axes[7] == -1.0

    def arrow_left_active(self):
        return self.joy.axes[6] == 1.0

    def arrow_right_active(self):
        return self.joy.axes[6] == -1.0


class JoyInput(object):
    def __init__(self):
        rospy.init_node('joy_input')
        rospy.Subscriber("joy", ButtonHandler, self.Joy2Twist)
        rospy.Subscriber("joy", ButtonHandler, self.Joy2Manipulate)
        rospy.Subscriber("joy", ButtonHandler, self.Joy2Motor)
        self.joy_cmd_vel = rospy.Publisher('joy_cmd_vel', Twist)
        self.joy_cmd_manipulate = rospy.Publisher('joy_cmd_manipulate', String)
        self.joy_cmd_prismatic = rospy.Publisher('/mark43_pris/command', Float64)
        rospy.on_shutdown(self.Stop)
        self.Start()

    def Joy2Motor(self, joy):

        motors = []
        motors[Button.X] = ""
        motors[Button.A] = ""
        motors[Button.B] = ""

        buttons = ButtonHandler(joy)

        cmd = 0.0
        if buttons.arrow_left_active():
            cmd = 1.0
        elif buttons.arrow_right_active():
            cmd = -1.0

        if cmd != 0.0:
            if buttons.x_active():
                rospy.Publisher(motors[Button.X], Float64).publish(Float64(cmd))
            if buttons.a_active():
                rospy.Publisher(motors[Button.A], Float64).publish(Float64(cmd))
            if buttons.b_active():
                rospy.Publisher(motors[Button.B], Float64).publish(Float64(cmd))


    def Joy2Manipulate(self, joy):

        mani_cmd = ""

        buttons = ButtonHandler(joy)

        if buttons.lb_active() and buttons.a_active():
            mani_cmd = "normal"
        elif buttons.lb_active() and buttons.b_active():
            mani_cmd = "straight"
        elif buttons.lb_active() and buttons.x_active():
            mani_cmd = "walking"
        elif buttons.lb_active() and buttons.y_active():
            mani_cmd = "prepare"

        if mani_cmd == "":
            return

        self.joy_cmd_manipulate.publish(mani_cmd)

    def Joy2Twist(self, joy):

        buttons = ButtonHandler(joy)

        if buttons.rb_active():
            cmd_vx = joy.axes[1]
            cmd_vy = joy.axes[4]
            cmd_vth = joy.axes[0]
            joy_cmd = Twist()
            joy_cmd.linear.x = cmd_vx * 0.5  # maximum vx is 0.2 m/s
            joy_cmd.linear.y = cmd_vy * 0.2  # maximum vy is 0.2 m/s
            joy_cmd.angular.z = cmd_vth * 0.8  # maximum vth is 0.4 rad/s

            cmd_pris = joy.axes[6]
        else:
            joy_cmd = Twist()
            cmd_pris = 0
        self.joy_cmd_vel.publish(joy_cmd)
        self.joy_cmd_prismatic.publish(Float64(cmd_pris))
        rospy.loginfo(joy_cmd)

    def Start(self):
        print 'Start joyplay'
        self.joy_cmd_vel.publish(Twist())

    def Stop(self):
        print 'Stop joyplay'
        self.joy_cmd_vel.publish(Twist())


if __name__ == '__main__':
    try:
        JoyInput()
        rospy.spin()
    except    rospy.ROSInterruptException:
        rospy.loginfo("stop joyplay")
