#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandActionGoal


class RightArm:
    def __init__(self):
        self.gripper_client = actionlib.SimpleActionClient('/dynamixel/right_gripper_controller/gripper_action',
                                                           GripperCommandAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.gripper_client.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')


    def open(self):
        action_open = GripperCommandGoal()
        action_open.command.position = 0.6
        self.gripper_client.send_goal(action_open)
        self.gripper_client.wait_for_result()
        if self.gripper_client.get_state() == actionlib.SimpleGoalState.DONE:
            rospy.loginfo('opened')
        else:
            rospy.loginfo('error')

    def close(self):
        action_close = GripperCommandGoal()
        action_close.command.position = 0.0
        self.gripper_client.send_goal(action_close)
        self.gripper_client.wait_for_result()
        if self.gripper_client.get_state() == actionlib.SimpleGoalState.DONE:
            rospy.loginfo('opened')
        else:
            rospy.loginfo('error')


def main():
    arm = RightArm()
    arm.open()
    arm.close()

                        
if __name__ == '__main__':
      rospy.init_node('joint_trajectory_action_tester')
      main()
