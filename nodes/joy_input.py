#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoyInput(object):

    def __init__(self):
        rospy.init_node('joy_input')
        rospy.Subscriber("joy", Joy, self.Joy2Twist)
        rospy.Subscriber("joy", Joy, self.Joy2Manipulate)
        self.joy_cmd_vel = rospy.Publisher('joy_cmd_vel', Twist)
        self.joy_cmd_manipulate = rospy.Publisher('joy_cmd_manipulate', String)
        rospy.on_shutdown(self.Stop)
        self.Start()

    def Joy2Manipulate(self,joy):
        mani_cmd = String()
        if(joy.buttons[4] == 1 and joy.buttons[0] == 1 ):
            print 'LB + A = Normal'
            mani_cmd = "normal"
        elif(joy.buttons[4] == 1 and joy.buttons[1] == 1):
            print 'LB + B = Straight'
            mani_cmd = "straight"
        elif(joy.buttons[4] == 1 and joy.buttons[2] == 1):
            print 'LB + X = Walking'
            mani_cmd = "walking"
        elif(joy.buttons[4] == 1 and joy.buttons[3] == 1):
            print 'LB + Y = Prepare'
            mani_cmd = "prepare"
        else:
            return
        self.joy_cmd_manipulate.publish(mani_cmd)


    def Joy2Twist(self,joy):
        if joy.buttons[5] == 1:
            cmd_vx = joy.axes[1]
            cmd_vy = joy.axes[2]
            cmd_vth = joy.axes[0]
            joy_cmd = Twist()
            joy_cmd.linear.x = cmd_vx*0.4        #maximum vx is 0.2 m/s
            joy_cmd.linear.y = cmd_vy*0.4        #maximum vy is 0.2 m/s
            joy_cmd.angular.z = cmd_vth*1.0        #maximum vth is 0.4 rad/s
        elif joy.buttons[7] == 1:
            joy_cmd = Twist()
            joy_cmd.angular.x = 1                #for enable            
        else:
            joy_cmd = Twist()
        self.joy_cmd_vel.publish(joy_cmd)
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
