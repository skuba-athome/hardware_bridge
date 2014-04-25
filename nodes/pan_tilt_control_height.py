#! /usr/bin/python
import roslib
roslib.load_manifest('robot_connect')
import sys
import rospy
import tf
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Quaternion

class PanTiltControl(object):
	def __init__(self):

		#print 'Number of arguments:', len(sys.argv), 'arguments.'
		#print 'kinect_height:', float(sys.argv[1])
		rospy.init_node('PanTiltControl')
		self.PanTiltTransformBroadcaster = tf.TransformBroadcaster()
		self.pan_ang = 0.0;
		self.tilt_ang = 0.0;
		self.kinect_height = 1.27;
		#Servo params
		self.pan_scale = 1.0
		self.tilt_scale = -1.0
		self.pan_offset = -0.00153398078788
		self.tilt_offset = 0.0306796157577
		
		#self.kinect_height = float(sys.argv[1])########
		self.get_valid_height = True###########
		
		self.tilt_cmd = rospy.Publisher("/tilt_kinect/command",Float64)
		self.pan_cmd = rospy.Publisher("/pan_kinect/command",Float64)
		self.pris_cmd = rospy.Publisher("/mark43_pris/command",Float64)
		rospy.Subscriber("/tilt_kinect/state", JointState, self.pantilt_callback)
		rospy.Subscriber("/pan_kinect/state", JointState, self.pantilt_callback)
		rospy.Subscriber("/mark43_pris/state", JointState, self.pantilt_callback)
		rospy.Subscriber("/pan_tilt_cmd", Quaternion, self.pan_tilt_cmd_callback)
		rospy.Subscriber("/height_cmd", Float64, self.kinect_height_cmd_callback)
		rospy.spin()
	
	def kinect_height_cmd_callback(self,msg):
		height_real = (msg.data - 0.88)*25.13274/0.5
		h = Float64()
		h.data = height_real
		print h.data
		self.pris_cmd.publish(h)
		
	def pan_tilt_cmd_callback(self,q):
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
		elif pantilt_new.name == 'mark43_pris':
			height_raw = pantilt_new.current_pos
			#self.kinect_height = height_raw*0.5/25.13274 + 0.88 #1440 deg = 25.13rad
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
		rospy.loginfo(str((self.pan_ang,self.tilt_ang)))
			
if __name__ == '__main__':
	PanTiltControl()