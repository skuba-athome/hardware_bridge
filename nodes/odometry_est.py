#! /usr/bin/python

import rospy
import tf

from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, Quaternion, Pose2D, TransformStamped

class OdometryEstimation(object):
    def __init__(self):
        rospy.init_node('odometry_est')
        self.odomPose = Pose2D()

        self.vx_odom = 0.0
        self.vy_odom = 0.0
        self.w_odom = 0.0
        
        self.last_est_time = rospy.Time()
        self.init = True
        self.OdometryTransformBroadcaster = tf.TransformBroadcaster()
        self.pose_odom = rospy.Publisher("odom", Odometry, queue_size=1)
        rospy.Subscriber("base_vel", TwistWithCovarianceStamped, self.velOdom_callback)
        rospy.spin()
                
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

        odometry.child_frame_id = "base_link"
        odometry.twist.twist.linear.x = self.vx_odom
        odometry.twist.twist.linear.y = self.vy_odom
        odometry.twist.twist.angular.z = self.w_odom
        self.pose_odom.publish(odometry)
        
#---------------------------------------------------------------------------------#
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

if __name__ == '__main__':
    OdometryEstimation()
