#!/usr/bin/env python
import roslib
roslib.load_manifest('robot_connect')
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Quaternion, Pose2D, TransformStamped

odomPose = Pose2D()
estPose = Pose2D()
esticpPose = Pose2D()
icpPose = Pose2D()

global tick
tick = 0
def pose_est_icp_callback(data):
	global esticpPose
	esticpPose = data

def pose_est_callback(data):
	global estPose
	estPose = data
    
def pose_odom_callback(data):
	global odomPose
	odomPose = data

def pose_icp_callback(data):
	global icpPose
	global tick
	icpPose = data
	print str((0,tick,esticpPose.x,esticpPose.y,esticpPose.theta,estPose.x,estPose.y,estPose.theta,odomPose.x,odomPose.y,odomPose.theta,icpPose.x,icpPose.y,icpPose.theta,0))
	tick = 0
	
def joy_callback(joy):
	global tick
	if tick == 0 and joy.buttons[4] == 1:
		tick = 1
		
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pose_est_icp", Pose2D, pose_est_icp_callback)
    rospy.Subscriber("pose_est", Pose2D, pose_est_callback)
    rospy.Subscriber("pose_odom", Pose2D, pose_odom_callback)
    rospy.Subscriber("pose2D", Pose2D, pose_icp_callback)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
