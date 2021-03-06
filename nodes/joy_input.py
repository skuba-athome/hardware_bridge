#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float64
from include.ButtonHandler import Button, ButtonHandler

class JoyInput:
    def __init__(self):
        rospy.init_node('joy_input')
        rospy.Subscriber("joy", Joy, self.Joy2Twist)
        rospy.Subscriber("joy", Joy, self.Joy2Manipulate)
        #rospy.Subscriber("joy", Joy, self.Joy2Motor)
        self.joy_cmd_vel = rospy.Publisher('joy_cmd_vel', TwistStamped, queue_size=1)
        self.joy_cmd_plot = rospy.Publisher('joy_cmd_plot', TwistStamped, queue_size=1)
        self.joy_cmd_manipulate = rospy.Publisher('joy_cmd_manipulate', String, queue_size=1)
        self.joy_cmd_prismatic = rospy.Publisher('/mark43_pris/command', Float64, queue_size=1)
        rospy.on_shutdown(self.Stop)
        self.joy_cmd = TwistStamped()
        self.Start()
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            self.joy_cmd.header.stamp = rospy.Time.now()
            self.joy_cmd_plot.publish(self.joy_cmd)
            rate.sleep()

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
            print 'LB + A = Normal'
            mani_cmd = "normal"
        elif buttons.lb_active() and buttons.b_active():
            print 'LB + B = Straight'
            mani_cmd = "straight"
        elif buttons.lb_active() and buttons.x_active():
            print 'LB + X = Walking'
            mani_cmd = "walking"
        elif buttons.lb_active() and buttons.y_active():
            print 'LB + Y = Prepare'
            mani_cmd = "prepare"

        if mani_cmd == "":
            return

        self.joy_cmd_manipulate.publish(mani_cmd)

    def Joy2Twist(self, joy):
        buttons = ButtonHandler(joy)

        self.joy_cmd = TwistStamped()
        if buttons.rb_active():
            cmd_vx = joy.axes[1]
            cmd_vy = joy.axes[2]
            cmd_vth = joy.axes[0]
            self.joy_cmd.twist.linear.x = cmd_vx * 0.3  # maximum vx is 0.2 m/s
            # self.joy_cmd.twist.linear.y = cmd_vy * 0.3  # maximum vy is 0.2 m/s
            self.joy_cmd.twist.angular.z = cmd_vth * 0.7  # maximum vth is 0.4 rad/s

            cmd_pris = joy.axes[5]
        else:
            cmd_pris = 0
        #self.joy_cmd_prismatic.publish(Float64(cmd_pris))
        self.joy_cmd_vel.publish(self.joy_cmd)
        rospy.loginfo(self.joy_cmd)

    def Start(self):
        print 'Start joyplay'
        self.joy_cmd_vel.publish(TwistStamped())

    def Stop(self):
        print 'Stop joyplay'
        self.joy_cmd_vel.publish(TwistStamped())


if __name__ == '__main__':
    try:
        JoyInput()
    except    rospy.ROSInterruptException:
        rospy.loginfo("stop joyplay")
