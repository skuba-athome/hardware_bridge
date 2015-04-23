#!/usr/bin/env python

import rospy
import time
import sys

from math import pi
from geometry_msgs.msg import Twist, TwistStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from SerialDataGateway import SerialDataGateway

class BaseSerial(object):
	def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
		rospy.init_node('base_serial')
		rospy.Subscriber("joy_cmd_vel", TwistStamped, self.JoyTwist2Cmd)
		rospy.Subscriber("cmd_vel", Twist, self.Twist2Cmd)
		#self.base_imu = rospy.Publisher('imu', Imu)
		self.base_twist = rospy.Publisher('base_vel',TwistWithCovarianceStamped)
		rospy.on_shutdown(self.Stop)
		self.mySerialDataGateway = SerialDataGateway(port, baudrate, self.HandleReceivedLine)
		self.start_byte = '\xFF'
		self.stop_byte = '\x1F'
		self.start_byte_dis = '\x7F'
		self.stop_byte_dis = '\x0F'
		self.fdb_dx = 0
		self.fdb_dy = 0
		self.fdb_dth = 0
		self.fdb_vx = 0
		self.fdb_vy = 0
		self.fdb_vth = 0
		self.fdb_ax = 0
		self.fdb_ay = 0
		self.fdb_ath = 0

		self.fdb_acc_x = 0
		self.fdb_acc_y = 0
		self.fdb_gyro_z = 0
		
		
		self.cmd_vx = 0
		self.cmd_vy = 0
		self.cmd_vth = 0

		self.joy_enable = 0

		rate = rospy.Rate(30.0)
		self.Start()
		while not rospy.is_shutdown():
			tmp1 = self.signedint2twochar(self.cmd_vx)
			cmd_msg = tmp1
			tmp1 = self.signedint2twochar(self.cmd_vy)
			cmd_msg = cmd_msg +tmp1
			tmp1 = self.signedint2twochar(self.cmd_vth)
			cmd_msg = cmd_msg +tmp1
			self.mySerialDataGateway.Write(self.start_byte+cmd_msg+self.stop_byte)
			rate.sleep()

	def Twist2Cmd(self,vel):
		if self.joy_enable == 0:#debug self.joy_enable == 1
			self.cmd_vx = int(vel.linear.x*(2**15-1)/2.0)		#maximum robot cmd in vx is 2 m/s#if self.joy_enable == 1:
			self.cmd_vy = int(vel.linear.y*(2**15-1)/2.0)		#maximum robot cmd in vy is 2 m/s
			self.cmd_vth = int(vel.angular.z*(2**15-1)/6.0)		#maximum robot cmd in vth is 6 rad/s

	def JoyTwist2Cmd(self,twist):
		self.joy_enable = int(twist.twist.angular.x)
		if self.joy_enable == 0:#debug self.joy_enable == 0
			self.cmd_vx = int(twist.twist.linear.x*(2**15-1)/2.0)		#maximum robot cmd in vx is 2 m/s
			self.cmd_vy = int(twist.twist.linear.y*(2**15-1)/2.0)		#maximum robot cmd in vy is 2 m/s
			self.cmd_vth = int(twist.twist.angular.z*(2**15-1)/6.0)		#maximum robot cmd in vth is 6 rad/s

		#rospy.loginfo('Scaled cmd is: ' + str(self.cmd_vx) + ',' + str(self.cmd_vy) + ',' + str(self.cmd_vth))
		#rospy.loginfo('Scaled cmd is: ' + str((self.cmd_vx,self.cmd_vy,self.cmd_vth)))
		
		#print 'Return'+str((self.fdb_vx,self.fdb_vy,self.fdb_vth))

	def HandleReceivedLine(self,line):
		if len(line) == 7 and line[0]==self.start_byte:  #if len(line) == 19 and line[0]==self.start_byte: 
			#fdb_imu = Imu()
			#fdb_imu.header.stamp = rospy.Time.now()
			#fdb_imu.header.frame_id = "base_link"
			#fdb_imu.linear_acceleration.x = self.twochar2signedint16(line[1]+line[2])*4.0*9.80665/32767.0
			#fdb_imu.linear_acceleration.y = self.twochar2signedint16(line[3]+line[4])*4.0*9.80665/32767.0
			#fdb_imu.linear_acceleration_covariance[0] = 0.005
			#fdb_imu.linear_acceleration_covariance[4] = 0.005
			#fdb_imu.angular_velocity.z = self.twochar2signedint16(line[5]+line[6])*500.0*pi/180.0/32767.0
			#fdb_imu.angular_velocity_covariance[8] = 0.003
			#self.base_imu.publish(fdb_imu)

			#fdb_odom = TwistWithCovarianceStamped()
			#fdb_odom.header.stamp = rospy.Time.now()
			#fdb_odom.header.frame_id = "base_link"
			#fdb_odom.twist.twist.linear.x = self.twochar2signedint16(line[7]+line[8])*2.0/32767.0
			#fdb_odom.twist.twist.linear.y = self.twochar2signedint16(line[9]+line[10])*2.0/32767.0
			#fdb_odom.twist.twist.angular.z = self.twochar2signedint16(line[11]+line[12])*6.0/32767.0
			#fdb_odom.twist.covariance[0] = 0.001
			#fdb_odom.twist.covariance[7] = 0.001
			#fdb_odom.twist.covariance[35] = 0.0008
			#self.base_twist.publish(fdb_odom)
			
			#.............................for new test robot (mecanum)......................................
			fdb_odom = TwistWithCovarianceStamped()
			fdb_odom.header.stamp = rospy.Time.now()
			fdb_odom.header.frame_id = "base_link"
			fdb_odom.twist.twist.linear.x = self.twochar2signedint16(line[1]+line[2])*2.0/32767.0
			fdb_odom.twist.twist.linear.y = self.twochar2signedint16(line[3]+line[4])*2.0/32767.0
			fdb_odom.twist.twist.angular.z = self.twochar2signedint16(line[5]+line[6])*6.0/32767.0
			fdb_odom.twist.covariance[0] = 0.001
			fdb_odom.twist.covariance[7] = 0.001
			fdb_odom.twist.covariance[35] = 0.0008
			self.base_twist.publish(fdb_odom)
			
			
			#self.fdb_dx = self.twochar2signedint16(line[1]+line[2])*500.0/32767.0
			#self.fdb_dy = self.twochar2signedint16(line[3]+line[4])*500.0/32767.0
			#self.fdb_dth = self.twochar2signedint16(line[5]+line[6])*360.0/32767.0
			            
			#self.fdb_vx = self.twochar2signedint16(line[7]+line[8])*500.0/32767.0
			#self.fdb_vy = self.twochar2signedint16(line[9]+line[10])*500.0/32767.0
			#self.fdb_vth = self.twochar2signedint16(line[11]+line[12])*360.0/32767.0
			
			#self.fdb_ax = self.twochar2signedint16(line[13]+line[14])*2.0/32767.0
			#self.fdb_ay = self.twochar2signedint16(line[15]+line[16])*2.0/32767.0
			#self.fdb_ath = self.twochar2signedint16(line[17]+line[18])*6.0*180.0/3.14/32767.0

			
			#rospy.loginfo('Return is: ' + str((self.fdb_dx,self.fdb_dy,self.fdb_dth,self.fdb_vx,self.fdb_vy,self.fdb_vth,self.fdb_ax,self.fdb_ay,self.fdb_ath)))
			#rospy.loginfo('Return is: ' + str((self.fdb_dth,self.fdb_vth)))

	def signedint2twochar(self,data):
		temp = data%2**16
		return "%c"%('%02x'%(temp>>8)).decode('hex')+"%c"%('%02x'%(temp&0xff)).decode('hex')

	def twochar2signedint16(self,data):
		return (int(data.encode('hex'),16) + 2**15) % 2**16 - 2**15 
                
	def Start(self):
		print 'Start connection'
		self.mySerialDataGateway.Start()
		self.mySerialDataGateway.Write(self.start_byte+'\x00\x00\x00\x00\x00\x00'+self.stop_byte)

	def Stop(self):
		print 'Stop connection'
		self.mySerialDataGateway.Write(self.start_byte_dis+'\x00\x00\x00\x00\x00\x00'+self.stop_byte_dis)
		self.mySerialDataGateway.Stop()

if __name__ == '__main__':
	BaseSerial()
