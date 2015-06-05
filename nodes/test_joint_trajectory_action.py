#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class RightArm:
        def __init__(self):
            self.jta = actionlib.SimpleActionClient('/dynamixel/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            goal.trajectory.joint_names = ['right_shoulder_1_joint', 
                                           'right_shoulder_2_joint',
                                           'right_elbow_joint',
                                           'right_wrist_1_joint',
                                           'right_wrist_2_joint',
                                           'right_wrist_3_joint'
                                           ]

            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(3)                   

            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              

def main():
            arm = RightArm()
            arm.move_joint([0.8/1.75,0.0,1.57,0.0,0.0,0.0])

                        
if __name__ == '__main__':
      rospy.init_node('joint_trajectory_action_tester')
      main()
