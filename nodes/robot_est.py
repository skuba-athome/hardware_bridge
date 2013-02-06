#! /usr/bin/python
import roslib
roslib.load_manifest('robot_connect')
import rospy
import tf
from math import sin, cos, pi
from numpy import *
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Quaternion, Pose2D, TransformStamped

class RobotEstimation(object):
	def __init__(self):
		rospy.init_node('robot_est')
		self.odomPose = Pose2D()
		self.estPose = Pose2D()
		self.esticpPose = Pose2D()

		self.acc_x = 0.0
		self.acc_y = 0.0
		self.gyro_z = 0.0

		self.vx_odom = 0.0
		self.vy_odom = 0.0
		self.w_odom = 0.0

		self.vx_icp = 0.0
		self.vy_icp = 0.0
		self.w_icp = 0.0


		self.x = matrix([[0.], [0.]])
		self.P = matrix([[2., 0.], [0., 2.]]) # initial uncertainty
		self.R = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]) # measurement uncertainty
		self.Q = matrix([[1., 0.], [0., 1.]]) # process uncertainty
		
		self.vx_est = 0.0
		self.vy_est = 0.0
		self.w_est = 0.0

		self.vx_feedicp = 0.0
		self.vy_feedicp = 0.0
		self.w_feedicp = 0.0
		self.last_est_time = rospy.Time()
		self.init = True
		self.OdometryTransformBroadcaster = tf.TransformBroadcaster()
		
		#self.pose_est_icp = rospy.Publisher("pose_est_icp",Pose2D)
		self.pose_odom = rospy.Publisher("odom", Odometry)
		#self.pose_est = rospy.Publisher("pose_est",Pose2D)
		#self.feedicp_twist = rospy.Publisher("vel",TwistStamped)
		self.est_twist = rospy.Publisher("vel_est",TwistStamped)
		#rospy.Subscriber("vel_icp", TwistWithCovarianceStamped, self.twistICP_callback)
		#rospy.Subscriber("imu", Imu, self.IMU_callback)
		rospy.Subscriber("vel_odom", TwistWithCovarianceStamped, self.velOdom_callback)
		rospy.spin()
	
	def twistICP_callback(self,twist_icp):
		self.vx_icp = twist_icp.twist.twist.linear.x
		self.vy_icp = twist_icp.twist.twist.linear.y
		self.w_icp = twist_icp.twist.twist.angular.z
		self.R[0,0:2] = twist_icp.twist.covariance[0:2]
		self.R[1,0:2] = twist_icp.twist.covariance[6:8]

		#pub_est_vel = TwistStamped()
		#pub_est_vel.header.stamp = rospy.Time.now()
		#pub_est_vel.header.frame_id = 'base_link'
		#pub_est_vel.twist.linear.x = self.vx_est
		#pub_est_vel.twist.linear.y = self.vy_est
		#pub_est_vel.twist.angular.z = self.w_est
		#self.est_twist.publish(pub_est_vel)

	def IMU_callback(self,imu):
		self.acc_x = imu.linear_acceleration.x
		self.acc_y = imu.linear_acceleration.y
		self.gyro_z = imu.angular_velocity.z
		self.Q[0,0] = imu.linear_acceleration_covariance[0]
		self.Q[1,1] = imu.linear_acceleration_covariance[4]
				
	def velOdom_callback(self,vel_odom):
		timenow = vel_odom.header.stamp
		dt = (timenow - self.last_est_time).to_sec()

		if self.init:
			dt = 0
			self.init = False

		#update odom variable
		self.vx_odom = vel_odom.twist.twist.linear.x
		self.vy_odom = vel_odom.twist.twist.linear.y
		self.w_odom = vel_odom.twist.twist.angular.z
		#for mecanum robot test---------------------------------
		pub_est_vel = TwistStamped()
		pub_est_vel.header.stamp = rospy.Time.now()
		pub_est_vel.header.frame_id = 'base_link'
		pub_est_vel.twist.linear.x = self.vx_odom
		pub_est_vel.twist.linear.y = self.vy_odom
		pub_est_vel.twist.angular.z = self.w_odom
		self.est_twist.publish(pub_est_vel)
		
		#self.R[2,2] = vel_odom.twist.covariance[0]
		#self.R[3,3] = vel_odom.twist.covariance[7]

		#self.getVelEstimation_KF(dt)
		#self.publishVelFeedICP_CF(dt)
		
#----------------------------------Position calculation--------------------------#
		self.odomPose.x = self.odomPose.x + (self.vx_odom*cos(self.odomPose.theta) - self.vy_odom*sin(self.odomPose.theta))*dt
		self.odomPose.y = self.odomPose.y + (self.vx_odom*sin(self.odomPose.theta) + self.vy_odom*cos(self.odomPose.theta))*dt
		self.odomPose.theta = self.odomPose.theta + self.w_odom*dt
		
		#self.pose_odom.publish(self.odomPose) #publish odom pose
		
		
		
		q = tf.transformations.quaternion_from_euler(0, 0, self.odomPose.theta)
		quaternion = Quaternion(q[0], q[1], q[2], q[3])
		
		odometry = Odometry()
		odometry.header.frame_id = "odom"
		odometry.header.stamp = timenow
		odometry.pose.pose.position.x = self.odomPose.x
		odometry.pose.pose.position.y = self.odomPose.y
		odometry.pose.pose.position.z = 0
		odometry.pose.pose.orientation = quaternion #tf.transformations.quaternion_from_euler(0, 0, self.odomPose.theta)#quaternion

		odometry.child_frame_id = "base_link"
		odometry.twist.twist.linear.x = self.vx_odom
		odometry.twist.twist.linear.y = self.vy_odom
		odometry.twist.twist.angular.z = self.w_odom
		self.pose_odom.publish(odometry)
		

		#self.esticpPose.x = self.esticpPose.x + (self.vx_est*cos(self.esticpPose.theta) - self.vy_est*sin(self.esticpPose.theta))*dt
		#self.esticpPose.y = self.esticpPose.y + (self.vx_est*sin(self.esticpPose.theta) + self.vy_est*cos(self.esticpPose.theta))*dt
		#self.esticpPose.theta = self.esticpPose.theta + self.w_est*dt
		
		#self.pose_est_icp.publish(self.esticpPose) #publish KF pose estimation
 
		#self.estPose.x = self.estPose.x + (self.vx_feedicp*cos(self.estPose.theta) - self.vy_feedicp*sin(self.estPose.theta))*dt
		#self.estPose.y = self.estPose.y + (self.vx_feedicp*sin(self.estPose.theta) + self.vy_feedicp*cos(self.estPose.theta))*dt
		#self.estPose.theta = self.estPose.theta + self.w_feedicp*dt		
		
		#self.pose_est.publish(self.estPose) #publish CF pose estimation
		
#---------------------------------------------------------------------------------#

		#self.OdometryTransformBroadcaster.sendTransform(
		#	(self.esticpPose.x, self.esticpPose.y, 0),
		#	tf.transformations.quaternion_from_euler(0, 0, self.esticpPose.theta),
		#	timenow,
		#	#rospy.Time.now(),
		#	"base_link",
		#	"odom"
		#)
		self.OdometryTransformBroadcaster.sendTransform(
			(self.odomPose.x, self.odomPose.y, 0),
			q,
			#tf.transformations.quaternion_from_euler(0, 0, self.odomPose.theta),
			timenow,
			#rospy.Time.now(),
			"base_link",
			"odom"
		)
			
		self.last_est_time = timenow

	def publishVelFeedICP_CF(self,dt):
		vel_acc_x = self.vx_feedicp + self.acc_x*dt
		vel_acc_y = self.vy_feedicp + self.acc_y*dt

		self.vx_feedicp = self.vx_odom*0.07 + vel_acc_x*0.93
		self.vy_feedicp = self.vy_odom*0.07 + vel_acc_y*0.93
		self.w_feedicp = 1.0*self.gyro_z + 0.0*self.w_odom #just weight average for w_feedicp

		vel_feedicp = TwistStamped()
		vel_feedicp.header.stamp = rospy.Time.now()
		vel_feedicp.header.frame_id = 'base_link'
		vel_feedicp.twist.linear.x = self.vx_feedicp
		vel_feedicp.twist.linear.y = self.vy_feedicp
		vel_feedicp.twist.angular.z = self.w_feedicp
		self.feedicp_twist.publish(vel_feedicp)	
		
	def getVelEstimation_KF(self,dt):
		u = matrix([[dt*self.acc_x], [dt*self.acc_y]]) # external motion
		F = matrix([[1., 0.], [0., 1.]]) # next state function
		H = matrix([[1., 0.], [0., 1.], [1., 0.], [0., 1.]]) # measurement function
		I = matrix([[1., 0.], [0., 1.]]) # identity matrix
		Z = matrix([[self.vx_icp], [self.vy_icp], [self.vx_odom], [self.vy_odom]])
		# prediction
		self.x = (F*self.x) + u
		self.P = (F*self.P*F.T) + self.Q
		#measurement update
		y = Z - (H*self.x)
		S = (H*self.P*H.T) + self.R
		K = self.P*H.T*S.I
		self.x = self.x + (K*y)
		self.P = (I - (K*H))*self.P
		#return KF est.
		self.vx_est = self.x[0,0]
		self.vy_est = self.x[1,0]
		self.w_est = 1.0*self.gyro_z + 0.0*self.w_odom	#just weight average for w_est
		print self.R

if __name__ == '__main__':
	RobotEstimation()
