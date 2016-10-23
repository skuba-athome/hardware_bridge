#! /usr/bin/python

import rospy
import tf
from sensor_msgs.msg import Imu
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Quaternion, Pose2D, TransformStamped, PoseWithCovarianceStamped

class OdometryEstimation(object):
    def __init__(self):
        rospy.init_node('odometry_est')
        self.odomPose = Pose2D()

        self.vx_odom = 0.0
        self.vy_odom = 0.0
        self.w_odom = 0.0
        
        self.last_est_time = rospy.Time()
        self.init = True
        # self.OdometryTransformBroadcaster = tf.TransformBroadcaster()
        self.pose_odom = rospy.Publisher("odom", Odometry, queue_size=1)
        self.imu_msg = rospy.Publisher("imu", Imu, queue_size=1)
        rospy.Subscriber("base_vel", TwistWithCovarianceStamped, self.velOdom_callback)
        # rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odom_combined_callback)
        rospy.Subscriber("/imu/data", Imu, self.odom_combined_callback)

        rospy.spin()
                
    def odom_combined_callback(self, odomPose):
    	# print 1111
        print odomPose
        quaternion = (
            odomPose.orientation.x,
            odomPose.orientation.y,
            odomPose.orientation.z,
            odomPose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print roll, pitch, yaw

        # orientation = Quaternion()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        # type(pose) = geometry_msgs.msg.Pose
        odomPose.orientation.x = quaternion[0]
        odomPose.orientation.y = quaternion[1]
        odomPose.orientation.z = quaternion[2]
        odomPose.orientation.w = quaternion[3]
        # print orientation
        self.imu_msg.publish(odomPose)
    	# self.OdometryTransformBroadcaster.sendTransform(
         #    (odomPose.angular_velocity.x, odomPose.angular_velocity.y, 0),
         #    (orientation.x, orientation.y, orientation.z, orientation.w),
         #    #tf.transformations.quaternion_from_euler(0, 0, self.odomPose.theta),
         #    odomPose.header.stamp,
         #    #rospy.Time.now(),
         #    "base_link",
         #    "odom"
       	# )


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
        
#----------------------------------Position calculation--------------------------#
        self.odomPose.x = self.odomPose.x + (self.vx_odom*cos(self.odomPose.theta) - self.vy_odom*sin(self.odomPose.theta))*dt
        self.odomPose.y = self.odomPose.y + (self.vx_odom*sin(self.odomPose.theta) + self.vy_odom*cos(self.odomPose.theta))*dt
        self.odomPose.theta = self.odomPose.theta + self.w_odom*dt
        
        q = tf.transformations.quaternion_from_euler(0, 0, self.odomPose.theta)
        quaternion = Quaternion(q[0], q[1], q[2], q[3])
        
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = timenow
        odometry.pose.pose.position.x = self.odomPose.x
        odometry.pose.pose.position.y = self.odomPose.y
        odometry.pose.pose.position.z = 0
        odometry.pose.pose.orientation = quaternion
        odometry.pose.covariance = [0.01,  0.0,  0.0,  0.0,  0.0,  0.0,
                                    0.0,  0.01,  0.0,  0.0,  0.0,  0.0,
                                    0.0,   0.0, 10000,  0.0,  0.0,  0.0,
                                    0.0,   0.0,  0.0,  10000,  0.0,  0.0,
                                    0.0,   0.0,  0.0,  0.0,  10000,  0.0,
                                    0.0,   0.0,  0.0,  0.0,  0.0,  0.01]

        odometry.child_frame_id = "base_link"
        odometry.twist.twist.linear.x = self.vx_odom
        odometry.twist.twist.linear.y = self.vy_odom
        odometry.twist.twist.angular.z = self.w_odom
        odometry.twist.covariance = [0.01,  0.0,  0.0,  0.0,  0.0,  0.0,
                                    0.0,  0.01,  0.0,  0.0,  0.0,  0.0,
                                    0.0,   0.0, 10000,  0.0,  0.0,  0.0,
                                    0.0,   0.0,  0.0,  10000,  0.0,  0.0,
                                    0.0,   0.0,  0.0,  0.0,  10000,  0.0,
                                    0.0,   0.0,  0.0,  0.0,  0.0,  0.01]
        self.pose_odom.publish(odometry)
        
#---------------------------------------------------------------------------------#
        # self.OdometryTransformBroadcaster.sendTransform(
        #    (self.odomPose.x, self.odomPose.y, 0),
        #    q,
        #    rospy.Time.now(),
        #    "base_link",
        #    "odom"
        # )

        # self.OdometryTransformBroadcaster.sendTransform(
        #    (0, 0, 0),
        #    q,
        #    tf.transformations.quaternion_from_euler(0, 0, self.odomPose.theta),
        #    timenow,
        #    rospy.Time.now(),
        #    "odom",
        #    "odom_combined"
        # )
        self.last_est_time = timenow

if __name__ == '__main__':
    OdometryEstimation()
