#! /usr/bin/python
import roslib
roslib.load_manifest('robot_connect')
import sys
import rospy
import tf
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from dynamixel_controllers.srv import SetSpeed

class PanTiltControl(object):
	def __init__(self):

		#print 'Number of arguments:', len(sys.argv), 'arguments.'
		print 'kinect_height:', float(sys.argv[1])
		rospy.init_node('PanTiltControl')
		self.PanTiltTransformBroadcaster = tf.TransformBroadcaster()
		self.pan_ang = 0.0;
		self.tilt_ang = 0.0;
		#Servo params
		self.pan_scale = 1.0
		self.tilt_scale = -1.0
		self.pan_offset = -0.00153398078788
		self.tilt_offset = 0.0306796157577
		
		self.kinect_height = float(sys.argv[1])########
		self.get_valid_height = True###########
		
		self.tilt_cmd = rospy.Publisher("/tilt_kinect/command",Float64)
		self.pan_cmd = rospy.Publisher("/pan_kinect/command",Float64)
		rospy.Subscriber("/tilt_kinect/state", JointState, self.pantilt_callback)
		rospy.Subscriber("/pan_kinect/state", JointState, self.pantilt_callback)
		rospy.Subscriber("/pan_tilt_cmd", Quaternion, self.pan_tilt_cmd_callback)
		rospy.Subscriber("/height_cmd", Float64, self.kinect_height_cmd_callback)

		rospy.spin()
	
	def kinect_height_callback(self,msg):
		self.kinect_height = msg.data
		self.get_valid_height = True
		
	def pan_tilt_cmd_callback(self,q):
		rospy.wait_for_service('/pan_kinect/set_speed')
		rospy.wait_for_service('/tilt_kinect/set_speed')
		try:
			pan_speed = rospy.ServiceProxy('/pan_kinect/set_speed',SetSpeed)
			tilt_speed = rospy.ServiceProxy('/tilt_kinect/set_speed',SetSpeed)
			rePan = pan_speed(0.4)
			reTilt = tilt_speed(0.4)
		except rospy.ServiceException, e:
			print "Service Speed call failed %s"%e

		uselessroll, tilt, pan = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
		rospy.loginfo("cmd_angle: %s",str((pan,tilt)))
		tilt_cmd_servo = (tilt - self.tilt_offset)/self.tilt_scale
		pan_cmd_servo = (pan - self.pan_offset)/self.pan_scale
		self.tilt_cmd.publish(tilt_cmd_servo)
		self.pan_cmd.publish(pan_cmd_servo)
		
	def pantilt_callback(self,pantilt_new):
		if pantilt_new.name == 'pan_kinect':
			pan_ang_raw = pantilt_new.current_pos
			self.pan_ang = self.pan_scale*pan_ang_raw + self.pan_offset
		elif pantilt_new.name == 'tilt_kinect':
			tilt_ang_raw = pantilt_new.current_pos
			self.tilt_ang = self.tilt_scale*tilt_ang_raw + self.tilt_offset
		else:
			rospy.logwarn("invalid servo name--> %s",str(pantilt_new.name));
		if (self.get_valid_height == True):	
			self.PanTiltTransformBroadcaster.sendTransform(
				(0.05, 0, self.kinect_height),
				tf.transformations.quaternion_from_euler(0, 0, self.pan_ang),
				rospy.Time.now(),
				"pan_link",
				"base_link"
			)
			self.PanTiltTransformBroadcaster.sendTransform(
				(0, 0, 0.025),
				tf.transformations.quaternion_from_euler(0, self.tilt_ang, 0),
				rospy.Time.now(),
				"tilt_link",
				"pan_link"
			)
			self.PanTiltTransformBroadcaster.sendTransform(
				(0, 0, 0.08),
				tf.transformations.quaternion_from_euler(0, 0, 0),
				rospy.Time.now(),
				"camera_link",
				"tilt_link"
			)	
			self.PanTiltTransformBroadcaster.sendTransform(
				(0.03, -0.15, self.kinect_height-0.15),
				tf.transformations.quaternion_from_euler(0, 0, 0),
				rospy.Time.now(),
				"mani_link",
				"base_link"
			)		
		rospy.loginfo(str((self.pan_ang,self.tilt_ang)))
			
if __name__ == '__main__':
	PanTiltControl()